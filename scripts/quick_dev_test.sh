#!/usr/bin/env bash
# 快速开发测试 — 只跑指定场景，跳过渲染，输出 JSON + 报告
# 用法: bash quick_dev_test.sh [scenario1] [scenario2] ...
# 默认跑 S3 S7 N2 N4 N12 五个关键场景

set -eu
WS="$HOME/f2c_coverage_planner"
RESULT_DIR="${WS}/test_results/quick_$(date +%m%d_%H%M)"
mkdir -p "$RESULT_DIR"

set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export LD_LIBRARY_PATH="$WS/install/fields2cover/lib:$WS/install/lib:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

TEST_POLYGONS="$WS/src/yingshi_robot/test_polygons"
F2C_AREAS="$WS/src/yingshi_robot/config/f2c_areas"

declare -A SCENARIO_MAP=(
    ["S1"]="$TEST_POLYGONS/S1_S1_convex_rect.yaml"
    ["S2"]="$TEST_POLYGONS/S2_S2_L_shaped.yaml"
    ["S3"]="$TEST_POLYGONS/S3_S3_with_holes.yaml"
    ["S4"]="$TEST_POLYGONS/S4_S4_narrow_corridor.yaml"
    ["S5"]="$TEST_POLYGONS/S5_S5_irregular.yaml"
    ["S6"]="$TEST_POLYGONS/S6_S6_multi_region.yaml"
    ["S7"]="$TEST_POLYGONS/S7_S7_factory_workshop.yaml"
    ["S8"]="$F2C_AREAS/notched_10m_with_center_hole.yaml"
    ["N1"]="$TEST_POLYGONS/ring.yaml"
    ["N2"]="$TEST_POLYGONS/oblique_with_round_hole.yaml"
    ["N3"]="$TEST_POLYGONS/dense_shelves.yaml"
    ["N4"]="$TEST_POLYGONS/warehouse_mixed_shelves.yaml"
    ["N5"]="$TEST_POLYGONS/u_corridor.yaml"
    ["N6"]="$TEST_POLYGONS/l_shape_large.yaml"
    ["N7"]="$TEST_POLYGONS/gate_shape.yaml"
    ["N8"]="$TEST_POLYGONS/u_shape.yaml"
    ["N9"]="$TEST_POLYGONS/warehouse_shelves.yaml"
    ["N10"]="$TEST_POLYGONS/warehouse_horiz_shelves.yaml"
    ["N11"]="$TEST_POLYGONS/warehouse_shelves_with_hole.yaml"
    ["N12"]="$TEST_POLYGONS/large_warehouse.yaml"
    ["N13"]="$TEST_POLYGONS/rect_multi_obstacles.yaml"
)

SCENARIOS=("${@:-S3 S7 N2 N4 N12}")

cleanup_ros() {
    pkill -f "polygon_planner_node" 2>/dev/null || true
    sleep 1
}

echo "⚡ Quick Dev Test: ${SCENARIOS[*]}"
echo "   输出: $RESULT_DIR"
echo ""

ERRORS=0
COVERAGE_THRESHOLD=0.99

for NAME in "${SCENARIOS[@]}"; do
    YAML="${SCENARIO_MAP[$NAME]:-}"
    if [ -z "$YAML" ] || [ ! -f "$YAML" ]; then
        echo "❌ [$NAME] 未知场景或文件不存在: $YAML"
        ERRORS=$((ERRORS + 1))
        continue
    fi

    echo -n "  [$NAME] "
    cleanup_ros

    LOG_FILE="$RESULT_DIR/${NAME}_planner.log"

    # 启动 planner，stderr 也重定向到 log
    ros2 run yingshi_robot polygon_planner_node --ros-args \
        -p use_planner_core:=true \
        -p robot_width:=0.75 \
        -p coverage_width:=0.90 \
        -p mid_hl_width_ratio:=0.20 \
        -p no_hl_width_ratio:=0.0 \
        -p swath_angle_optimization:=true \
        -p use_sweep_decomp:=true \
        -p eval_enable_report:=true \
        -p eval_use_grid_method:=true \
        > "$LOG_FILE" 2>&1 &
    PLANNER_PID=$!
    sleep 1

    # 用 ros2 topic echo --once 主动等路径（替代盲等）
    python3 - "$NAME" "$YAML" "$LOG_FILE" "$RESULT_DIR" << 'PYQUICK'
import sys, time, yaml, json, os, re, subprocess, signal

name = sys.argv[1]
yaml_path = sys.argv[2]
log_file = sys.argv[3]
result_dir = sys.argv[4]

# 读取多边形
with open(yaml_path) as f:
    area = yaml.safe_load(f)

# 发布 polygon + holes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

rclpy.init()
node = rclpy.create_node(f'quick_{name}')
plan_received = False

def on_path(msg):
    global plan_received
    if len(msg.poses) > 0: plan_received = True

node.create_subscription(Path, '/planned2_path_1', on_path, 10)
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

poly_msg = Polygon()
for p in area['polygon']:
    poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

holes_msg = Polygon()
for hi, hole in enumerate(area.get('holes', [])):
    if hi > 0:
        holes_msg.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in hole:
        holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

# 发布，然后主动 spin 等 plan
holes_pub.publish(holes_msg)
rclpy.spin_once(node, timeout_sec=0.1)
time.sleep(0.2)
poly_pub.publish(poly_msg)

# 等 plan 出现（最多 30s）
for _ in range(60):
    rclpy.spin_once(node, timeout_sec=0.5)
    if plan_received:
        break

if not plan_received:
    print("FAIL (no plan)")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

# 等评估完成（读 log 文件中的 PLANNERCORE_COMPLETE）
eval_done = False
for _ in range(60):
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.1)
    if os.path.exists(log_file):
        with open(log_file) as f:
            if 'PLANNERCORE_COMPLETE' in f.read():
                eval_done = True
                break

node.destroy_node()
rclpy.shutdown()

# 解析日志
text = ''
if os.path.exists(log_file):
    with open(log_file) as f:
        text = f.read()

def extract(pat, text, cast=float, flags=0):
    matches = re.findall(pat, text, flags=flags)
    return cast(matches[-1]) if matches else None

cov = extract(r'覆盖率[:\s]*([\d.]+)%', text) or 0
score = extract(r'综合得分[:\s]*([\d.]+)', text) or 0
dist = extract(r'路径总长[:\s]*([\d.]+)', text) or 0
overlap = extract(r'重叠率[:\s]*([\d.]+)%', text) or 0
turns = extract(r'^转弯次数:\s*(\d+)\s*$', text, int, re.MULTILINE) or 0
cells_count = extract(r'\[merge\] \d+→(\d+) cells', text, int) or 0
merged = extract(r'\[merge\].*?(\d+) pairs merged', text, int) or 0

# 从 vis JSON 读取 cell 数
vis_json = '/tmp/f2c_vis_polygon_1.json'
actual_cells = 0
if os.path.exists(vis_json):
    with open(vis_json) as f:
        vd = json.load(f)
    actual_cells = len(vd.get('cells', []))

# 保存简化 JSON
data = {
    'scenario': name,
    'cells': actual_cells,
    'merged': merged,
    'cov': cov,
    'score': score,
    'dist': dist,
    'overlap': overlap,
    'turns': turns,
}

data_file = f'{result_dir}/{name}_quick.json'
with open(data_file, 'w') as f:
    json.dump(data, f, indent=2)

gate = 'PASS' if cov >= 99.0 else 'FAIL'
print(f"{gate}  cells={actual_cells}  merge={merged}  cov={cov:.2f}%  score={score:.1f}  dist={dist:.0f}m  overlap={overlap:.1f}%  turns={turns}")
PYQUICK

    kill $PLANNER_PID 2>/dev/null || true
    wait $PLANNER_PID 2>/dev/null || true
done

# 简易报告
echo ""
echo "=============================="
echo "  Quick Dev Test 报告"
echo "=============================="
printf "%-6s %6s %5s %7s %6s %7s %7s\n" 场景 cells merge 覆盖率 得分 路径长 重叠率
for NAME in "${SCENARIOS[@]}"; do
    DF="$RESULT_DIR/${NAME}_quick.json"
    if [ -f "$DF" ]; then
        python3 -c "
import json
with open('$DF') as f: d=json.load(f)
print(f\"{d['scenario']:6s} {d['cells']:5d} {d['merged']:4d}  {d['cov']:5.2f}% {d['score']:5.1f} {d['dist']:6.0f}m {d['overlap']:5.1f}%\")
"
    fi
done
echo ""
echo "报告目录: $RESULT_DIR"
echo "Done."
