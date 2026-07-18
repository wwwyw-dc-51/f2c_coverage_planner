#!/usr/bin/env bash
# F2C 8场景批量测试 v2 — 每场景独立隔离，避免 topic 串扰
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
RESULT_DIR="${1:-$WS/test_results/batch_$(date +%m%d_%H%M)}"
mkdir -p "$RESULT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export LD_LIBRARY_PATH="$WS/install/fields2cover/lib:$WS/install/lib:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

TEST_POLYGONS="$WS/src/yingshi_robot/test_polygons"
F2C_AREAS="$WS/src/yingshi_robot/config/f2c_areas"
declare -A SCENARIOS=(
    ["S1"]="$TEST_POLYGONS/S1_S1_convex_rect.yaml"
    ["S2"]="$TEST_POLYGONS/S2_S2_L_shaped.yaml"
    ["S3"]="$TEST_POLYGONS/S3_S3_with_holes.yaml"
    ["S4"]="$TEST_POLYGONS/S4_S4_narrow_corridor.yaml"
    ["S5"]="$TEST_POLYGONS/S5_S5_irregular.yaml"
    ["S6"]="$TEST_POLYGONS/S6_S6_multi_region.yaml"
    ["notched"]="$F2C_AREAS/notched_10m_with_center_hole.yaml"
    ["S7"]="$TEST_POLYGONS/S7_S7_factory_workshop.yaml"
)

PLANNER_PARAMS=(
    -p use_planner_core:=true
    -p robot_width:=0.75
    -p coverage_width:=0.90
    -p mid_hl_width_ratio:=0.20
    -p no_hl_width_ratio:=0.0
    -p min_hole_area:=0.0
    -p traversability_enabled:=true
    -p cspace_clearance_margin:=0.0
    -p max_excluded_area_ratio:=0.05
    -p decomposition_angle:=0.0
    -p swath_endpoint_shrink_distance:=0.03
    -p min_swath_length:=0.5
    -p max_diff_curv:=0.3
    -p path_resolution:=0.1
    -p boundary_type:=closed
    -p use_optimized_planner:=true
    -p swath_angle_optimization:=true
    -p decomposition_enabled:=true
    -p filter_tiny_cells:=true
    -p use_sweep_decomp:=true
    -p merge_angle_threshold:=60.0
    -p swath_order_type:=boustrophedon
    -p path_simplify_enabled:=true
    -p path_simplify_tolerance:=0.05
    -p path_simplify_turn_threshold:=0.15
    -p turn_planner_type:=direct
    -p swath_overlap_ratio:=0.03
    -p eval_enable_report:=true
    -p eval_use_grid_method:=true
    -p eval_grid_resolution:=0.1
    -p eval_coverage_threshold:=0.99
)

cleanup_ros() {
    # 杀掉所有相关进程
    pkill -f "polygon_planner_node" 2>/dev/null || true
    pkill -f "test_publisher" 2>/dev/null || true
    pkill -f "capture_" 2>/dev/null || true
    sleep 2
}

echo "========================================="
echo "  F2C 8场景批量测试 v2"
echo "  输出: $RESULT_DIR"
echo "========================================="

ERRORS=0
COVERAGE_THRESHOLD=0.99  # 产品验收门槛 99%
VIS_JSON="/tmp/f2c_vis_polygon_1.json"
GRID_JSON="/tmp/f2c_grid_polygon_1.json"

for NAME in S1 S2 S3 S4 S5 S6 S7 notched; do
    YAML="${SCENARIOS[$NAME]}"
    echo ""
    echo "=== [$NAME] $(date +%H:%M:%S) ==="

    cleanup_ros

    LOG_FILE="$RESULT_DIR/${NAME}_planner.log"
    DATA_FILE="$RESULT_DIR/${NAME}_data.json"

    # 每场景从干净产物开始，避免复用上一场景或同目录旧批次的数据。
    rm -f "$VIS_JSON" "$GRID_JSON" "$DATA_FILE" \
        "${GRID_JSON%.json}"_component_*.json \
        "$RESULT_DIR/${NAME}_grid.json" \
        "${RESULT_DIR}/${NAME}_grid_component_"*.json \
        "$RESULT_DIR/${NAME}_coverage.png" \
        "$RESULT_DIR/${NAME}_cells.png" \
        "$RESULT_DIR/${NAME}_connections.png"

    # 启动 planner
    ros2 run yingshi_robot polygon_planner_node --ros-args "${PLANNER_PARAMS[@]}" > "$LOG_FILE" 2>&1 &
    PLANNER_PID=$!
    echo "  Planner PID: $PLANNER_PID"

    # 等待 planner 就绪
    sleep 3
    if ! kill -0 $PLANNER_PID 2>/dev/null; then
        echo "  ERROR: planner crashed"
        cat "$LOG_FILE" | tail -5
        ERRORS=$((ERRORS + 1))
        continue
    fi

    # 一步完成: 发布 + 订阅路径 + 等待评估
    set +e
    python3 - "$NAME" "$YAML" "$LOG_FILE" "$RESULT_DIR" "$VIS_JSON" "$GRID_JSON" << 'PYTEST'
import sys, time, yaml, json, os, re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

HOLE_SEP = 1e10
name = sys.argv[1]
yaml_path = sys.argv[2]
log_file = sys.argv[3]
result_dir = sys.argv[4]
vis_json = sys.argv[5]
grid_json = sys.argv[6]

rclpy.init()
node = rclpy.create_node(f'test_{name}')

# 数据容器
path_data = []
component_path_data = {}
plan_received = False

def on_path(msg):
    global path_data, plan_received
    if not plan_received and len(msg.poses) > 0:
        path_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
        plan_received = True

node.create_subscription(Path, '/planned2_path_1', on_path, 10)

def make_component_callback(component_index):
    def on_component_path(msg):
        global plan_received
        if len(msg.poses) > 0 and component_index not in component_path_data:
            component_path_data[component_index] = [
                {'x': p.pose.position.x, 'y': p.pose.position.y}
                for p in msg.poses
            ]
            plan_received = True
    return on_component_path

component_subscriptions = []
for component_index in range(1, 65):
    component_subscriptions.append(node.create_subscription(
        Path,
        f'/planned2_path_1_component_{component_index}',
        make_component_callback(component_index),
        10))
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

# 读取多边形
with open(yaml_path) as f:
    area = yaml.safe_load(f)

poly_msg = Polygon()
for p in area['polygon']:
    poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

holes_msg = Polygon()
for hi, hole in enumerate(area.get('holes', [])):
    if hi > 0:
        holes_msg.points.append(Point32(x=HOLE_SEP, y=0.0, z=0.0))
    for p in hole:
        holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

time.sleep(1.0)

# 发布(带重试)
for attempt in range(5):
    holes_pub.publish(holes_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.3)
    poly_pub.publish(poly_msg)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.5)
        if plan_received:
            break
    if plan_received:
        print(f'  Polygon published, plan received (attempt {attempt+1})')
        break
    print(f'  Retry {attempt+1}...')

# 等待评估完成(读取planner日志)
print('  Waiting for evaluation...')
eval_found = False
for i in range(120):
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    if os.path.exists(log_file):
        with open(log_file) as f:
            content = f.read()
            component_matches = re.findall(
                r'CSPACE_REPORT polygon=1 valid=true .*?components=(\d+)',
                content)
            expected_evaluations = (
                int(component_matches[-1]) if component_matches else 1)
            completion_matches = re.findall(
                r'PLANNERCORE_COMPLETE polygon=1 components=(\d+)',
                content)
            paths_complete = (
                plan_received and (
                    expected_evaluations == 1 or
                    len(component_path_data) >= expected_evaluations))
            if (completion_matches and
                    int(completion_matches[-1]) == expected_evaluations and
                    paths_complete):
                print(f'  Eval complete after {i+1}s')
                eval_found = True
                break

# 解析结果
result = {}
cov = 0.0
score = 0.0
text = ''
if os.path.exists(log_file):
    with open(log_file) as f:
        text = f.read()
if eval_found:
    def extract(pat, text, cast=float, flags=0):
        # 多 cell 场景会输出多组评估，取最后一组（最终汇总）
        matches = re.findall(pat, text, flags=flags)
        if matches:
            val = matches[-1]
            return cast(val) if cast != int else int(val)
        return None

    result = {
        'scenario': name,
        'coverage_rate': extract(r'覆盖率[:\s]*([\d.]+)%', text),
        'single_score': extract(r'综合得分[:\s]*([\d.]+)', text),
        'uncovered_area': extract(r'未覆盖面积[:\s]*([\d.]+)', text),
        'total_distance': extract(r'路径总长[:\s]*([\d.]+)', text),
        'work_ratio': extract(r'有效工作比[:\s]*([\d.]+)%', text),
        # 用 $ 锚定行尾整数，排除评分行的 "转弯次数: 5.678/15"
        'turn_count': extract(r'^转弯次数:\s*(\d+)\s*$', text, int, re.MULTILINE),
        'overlap_rate': extract(r'重叠率[:\s]*([\d.]+)%', text),
        'planning_time_ms': extract(r'规划耗时[:\s]*([\d.]+)\s*ms', text),
    }
    m = re.findall(r'目标净面积[:\s]*(-?[\d.]+)', text)
    if m: result['net_area'] = float(m[-1])

    cov = result.get('coverage_rate', 0) or 0
    score = result.get('single_score', 0) or 0

cspace = {}
cspace_match = re.findall(
    r'CSPACE_REPORT polygon=1 valid=true '
    r'original_area=([\d.eE+-]+) reachable_area=([\d.eE+-]+) '
    r'excluded_area=([\d.eE+-]+) excluded_ratio=([\d.eE+-]+) '
    r'max_ratio=([\d.eE+-]+) components=(\d+) '
    r'requires_repositioning=(true|false) gate=(PASS|FAIL)',
    text)
if cspace_match:
    values = cspace_match[-1]
    cspace = {
        'valid': True,
        'original_area': float(values[0]),
        'reachable_area': float(values[1]),
        'excluded_area': float(values[2]),
        'excluded_ratio': float(values[3]),
        'max_ratio': float(values[4]),
        'component_count': int(values[5]),
        'requires_repositioning': values[6] == 'true',
        'gate': values[7],
    }
elif 'CSPACE_REPORT polygon=1 valid=false' in text:
    cspace = {'valid': False}

component_evals = []
component_count = cspace.get('component_count', 1)
if eval_found and component_count > 1:
    component_patterns = {
        'coverage_rate': r'覆盖率[:\s]*([\d.]+)%',
        'single_score': r'综合得分[:\s]*([\d.]+)',
        'uncovered_area': r'未覆盖面积[:\s]*([\d.]+)',
        'total_distance': r'路径总长[:\s]*([\d.]+)',
        'work_ratio': r'有效工作比[:\s]*([\d.]+)%',
        'turn_count': r'^转弯次数:\s*(\d+)\s*$',
        'overlap_rate': r'重叠率[:\s]*([\d.]+)%',
        'planning_time_ms': r'规划耗时[:\s]*([\d.]+)\s*ms',
        'net_area': r'目标净面积[:\s]*(-?[\d.]+)',
    }
    component_values = {}
    for metric_name, pattern in component_patterns.items():
        flags = re.MULTILINE if metric_name == 'turn_count' else 0
        component_values[metric_name] = re.findall(pattern, text, flags)[-component_count:]
    if all(len(values) == component_count
           for values in component_values.values()):
        for component_index in range(component_count):
            component_eval = {}
            for metric_name, values in component_values.items():
                component_eval[metric_name] = (
                    int(values[component_index])
                    if metric_name == 'turn_count'
                    else float(values[component_index]))
            component_evals.append(component_eval)
        result = {}

# 从 planner 原生 JSON 读取完整路径（ROS topic 的简化路径点数太少，图对不上）
full_path = path_data
component_paths = [
    component_path_data[index]
    for index in sorted(component_path_data)
]
if os.path.exists(vis_json):
    try:
        with open(vis_json) as vf:
            vd = json.load(vf)
        native_pts = []
        if vd.get('path'):
            native_pts = vd['path']
        elif vd.get('component_paths'):
            native_components = vd['component_paths']
            if sum(map(len, native_components)) > sum(map(len, component_paths)):
                component_paths = native_components
        elif 'cells' in vd:
            # 多 cell 场景：拼接所有 cell 的路径
            for cell in vd.get('cells', []):
                native_pts.extend(cell.get('path', []))
        if len(native_pts) > len(full_path):
            full_path = native_pts
            print(f'  Using native JSON path: {len(full_path)} pts (ROS had {len(path_data)})')
    except Exception as e:
        print(f'  WARNING: native JSON read failed: {e}')

print(f'  {name}: cov={cov:.2f}% score={score:.1f} path_pts={len(full_path)}')

# 从 vis JSON 读取 cells + connections
vis_cells = []
vis_connections = []
vis_swaths = []
if os.path.exists(vis_json):
    try:
        with open(vis_json) as vf:
            vd = json.load(vf)
        vis_cells = vd.get('cells', [])
        vis_connections = vd.get('connections', [])
        vis_swaths = vd.get('swaths', [])
    except Exception as e:
        print(f'  WARNING: cells JSON read failed: {e}')

# 保存数据
merged = {
    'scenario': name,
    'path': full_path,
    'component_paths': component_paths,
    'swaths': vis_swaths,
    'eval': result,
    'component_evals': component_evals,
    'cspace': cspace,
    'cells': vis_cells,
    'connections': vis_connections,
    'batch_status': {
        'plan_received': plan_received,
        'evaluation_completed': eval_found,
        'visualization_artifact_created': os.path.exists(vis_json),
        'grid_artifact_created': (
            os.path.exists(grid_json)
            if component_count == 1
            else all(os.path.exists(grid_json.replace(
                '.json', f'_component_{index}.json'))
                for index in range(1, component_count + 1))),
    },
}
data_file = f'{result_dir}/{name}_data.json'
with open(data_file, 'w') as f:
    json.dump(merged, f, indent=2, ensure_ascii=False)

node.destroy_node()
rclpy.shutdown()
PYTEST
    CAPTURE_STATUS=$?
    set -e

    # 等待 planner 自然结束(评估完成后planner继续spin)
    sleep 2
    # 保存每场景独立的 grid JSON（避免后续渲染用错数据）
    if [ -f "$GRID_JSON" ]; then
        cp "$GRID_JSON" "$RESULT_DIR/${NAME}_grid.json"
    fi
    for COMPONENT_GRID in "${GRID_JSON%.json}"_component_*.json; do
        if [ -f "$COMPONENT_GRID" ]; then
            COMPONENT_SUFFIX="${COMPONENT_GRID##*_component_}"
            cp "$COMPONENT_GRID" \
                "$RESULT_DIR/${NAME}_grid_component_${COMPONENT_SUFFIX}"
        fi
    done
    kill $PLANNER_PID 2>/dev/null || true
    wait $PLANNER_PID 2>/dev/null || true

    echo "  $NAME done."

    if [ "$CAPTURE_STATUS" -ne 0 ]; then
        echo "  ERROR: $NAME capture process failed (exit=$CAPTURE_STATUS)"
        ERRORS=$((ERRORS + 1))
    elif ! python3 "$WS/scripts/batch_result_gate.py" "$DATA_FILE" \
            --coverage-threshold "$COVERAGE_THRESHOLD"; then
        ERRORS=$((ERRORS + 1))
    fi
done

echo ""
echo "========================================="
echo "  测试完成! 生成可视化..."
echo "========================================="

cleanup_ros

# 渲染所有场景
for NAME in S1 S2 S3 S4 S5 S6 S7 notched; do
    YAML="${SCENARIOS[$NAME]}"
    DATA_FILE="$RESULT_DIR/${NAME}_data.json"
    PNG_FILE="$RESULT_DIR/${NAME}_coverage.png"
    if [ -f "$DATA_FILE" ]; then
        GRID_JSON="$RESULT_DIR/${NAME}_grid.json"
        [ ! -f "$GRID_JSON" ] && GRID_JSON=""  # 无grid文件则回退Python自行采样
        python3 "$WS/scripts/render_coverage.py" "$NAME" "$YAML" "$DATA_FILE" "$PNG_FILE" "$GRID_JSON" 2>&1

        # Cell 划分图
        CELLS_PNG="$RESULT_DIR/${NAME}_cells.png"
        python3 "$WS/scripts/render_coverage.py" "$NAME" "$YAML" "$DATA_FILE" "$CELLS_PNG" --mode cells 2>&1

        # Cell 间连接图
        CONN_PNG="$RESULT_DIR/${NAME}_connections.png"
        python3 "$WS/scripts/render_coverage.py" "$NAME" "$YAML" "$DATA_FILE" "$CONN_PNG" --mode connections 2>&1
    fi
done

# 生成报告
python3 - "$RESULT_DIR" << 'PYREPORT'
import sys, json, os, datetime
rd = sys.argv[1]

lines = []
lines.append("")
lines.append("="*70)
lines.append("  F2C 8场景覆盖规划 — 批量测试报告")
lines.append("="*70)
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
lines.append(f"生成时间: {now}")
lines.append("")
lines.append("| 场景 | 面积 | 覆盖率 | 得分 | 未覆盖 | 路径长 | 重叠率 | 耗时 |")
lines.append("|:----:|:----:|:-----:|:----:|:-----:|:-----:|:-----:|:---:|")
for n in ['S1','S2','S3','S4','S5','S6','S7','notched']:
    df = f'{rd}/{n}_data.json'
    if os.path.exists(df):
        with open(df) as f:
            data = json.load(f)
            e = data.get('eval', {})
            component_evals = data.get('component_evals', [])
            if component_evals:
                e = {
                    'net_area': sum(v.get('net_area', 0) or 0 for v in component_evals),
                    'coverage_rate': min(v.get('coverage_rate', 0) or 0 for v in component_evals),
                    'single_score': min(v.get('single_score', 0) or 0 for v in component_evals),
                    'uncovered_area': sum(v.get('uncovered_area', 0) or 0 for v in component_evals),
                    'total_distance': sum(v.get('total_distance', 0) or 0 for v in component_evals),
                    'overlap_rate': max(v.get('overlap_rate', 0) or 0 for v in component_evals),
                    'planning_time_ms': sum(v.get('planning_time_ms', 0) or 0 for v in component_evals),
                }
        a = e.get('net_area',0) or 0
        c = e.get('coverage_rate',0) or 0
        s = e.get('single_score',0) or 0
        u = e.get('uncovered_area',0) or 0
        d = e.get('total_distance',0) or 0
        o = e.get('overlap_rate',0) or 0
        t = e.get('planning_time_ms',0) or 0
        lines.append(f"| {n} | {a:.0f} | {c:.2f}% | {s:.1f} | {u:.2f} | {d:.1f} | {o:.1f}% | {t:.0f} |")
lines.append("")
lines.append(f"报告目录: {rd}")

report_text = "\n".join(lines)
print(report_text)

# 同时写入报告文件
report_path = os.path.join(rd, "batch_report.txt")
with open(report_path, "w", encoding="utf-8") as rf:
    rf.write(report_text)
print(f"\n报告已保存: {report_path}")
PYREPORT

echo ""
echo "Done! Report: $RESULT_DIR"

# 退出码：有错误或覆盖低于门槛则非零
if [ $ERRORS -gt 0 ]; then
    echo "❌ $ERRORS scenario(s) failed (crash/timeout/missing data)"
    exit 1
fi
echo "✅ All 7 scenarios passed, no failures."
exit 0
