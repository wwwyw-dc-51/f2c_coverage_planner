#!/usr/bin/env bash
# F2C 6场景批量测试 + 可视化报告生成
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
RESULT_DIR="$WS/test_results/batch_$(date +%m%d_%H%M)"
mkdir -p "$RESULT_DIR"

# Source ROS2
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

TEST_POLYGONS="$WS/src/yingshi_robot/test_polygons"
SCENARIOS=(
    "S1:$TEST_POLYGONS/S1_S1_convex_rect.yaml"
    "S2:$TEST_POLYGONS/S2_S2_L_shaped.yaml"
    "S3:$TEST_POLYGONS/S3_S3_with_holes.yaml"
    "S4:$TEST_POLYGONS/S4_S4_narrow_corridor.yaml"
    "S5:$TEST_POLYGONS/S5_S5_irregular.yaml"
    "S6:$TEST_POLYGONS/S6_S6_multi_region.yaml"
)

echo "========================================="
echo "  F2C 6场景批量测试"
echo "  输出目录: $RESULT_DIR"
echo "========================================="

for entry in "${SCENARIOS[@]}"; do
    NAME="${entry%%:*}"
    YAML="${entry##*:}"

    echo ""
    echo "=== [$NAME] Testing... ==="

    LOG_FILE="$RESULT_DIR/${NAME}_planner.log"
    PATH_FILE="$RESULT_DIR/${NAME}_path.json"

    # 启动 planner
    ros2 run yingshi_robot polygon_planner_node --ros-args \
      -p robot_width:=0.95 \
      -p coverage_width:=0.45 \
      -p mid_hl_width_ratio:=0.20 \
      -p no_hl_width_ratio:=0.0 \
      -p min_hole_area:=1.0 \
      -p decomposition_angle:=0.0 \
      -p swath_endpoint_shrink_distance:=0.03 \
      -p min_swath_length:=0.5 \
      -p max_diff_curv:=0.3 \
      -p path_resolution:=0.1 \
      -p boundary_type:=closed \
      -p use_optimized_planner:=true \
      -p swath_angle_optimization:=true \
      -p decomposition_angle_optimization:=false \
      -p decomposition_enabled:=true \
      -p filter_tiny_cells:=true \
      -p path_simplify_enabled:=true \
      -p path_simplify_tolerance:=0.05 \
      -p path_simplify_turn_threshold:=0.15 \
      -p turn_planner_type:=direct \
      -p swath_overlap_ratio:=0.03 \
      -p eval_enable_report:=true \
      -p eval_use_grid_method:=true \
      -p eval_grid_resolution:=0.1 \
      -p eval_coverage_threshold:=0.995 > "$LOG_FILE" 2>&1 &

    PLANNER_PID=$!
    sleep 2

    # 启动路径捕获器 (后台)
    python3 - "$NAME" "$PATH_FILE" > "$RESULT_DIR/${NAME}_capture.log" 2>&1 << 'PYCAPTURE' &
import sys, time, json, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

name = sys.argv[1]
out_file = sys.argv[2]

rclpy.init()
node = rclpy.create_node(f'capture_{name}')

path_data = []
def on_path(msg):
    global path_data
    if not path_data:
        path_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]

node.create_subscription(Path, '/planned2_path_1', on_path, 10)

# Wait up to 60s for data
for i in range(120):
    rclpy.spin_once(node, timeout_sec=0.5)
    if path_data:
        break

with open(out_file, 'w') as f:
    json.dump({'path': path_data}, f)
print(f'Path data saved: {len(path_data)} points')

node.destroy_node()
rclpy.shutdown()
PYCAPTURE
    CAPTURE_PID=$!

    # 发布多边形
    python3 - "$YAML" >> "$LOG_FILE" 2>&1 << 'PYPUB'
import sys, time, yaml
import rclpy
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

yaml_path = sys.argv[1]
with open(yaml_path) as f:
    area = yaml.safe_load(f)

HOLE_SEP = 1e10
rclpy.init()
node = rclpy.create_node('publisher')
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

plan_ok = False
def on_plan(msg):
    global plan_ok
    if len(msg.poses) > 0:
        plan_ok = True
node.create_subscription(Path, '/planned2_path_1', on_plan, 10)

time.sleep(1.0)

poly_msg = Polygon()
for p in area['polygon']:
    poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

holes_msg = Polygon()
for hi, hole in enumerate(area.get('holes', [])):
    if hi > 0:
        holes_msg.points.append(Point32(x=HOLE_SEP, y=0.0, z=0.0))
    for p in hole:
        holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

for attempt in range(5):
    holes_pub.publish(holes_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)
    poly_pub.publish(poly_msg)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.5)
        if plan_ok:
            break
    if plan_ok:
        break

time.sleep(3.0)
node.destroy_node()
rclpy.shutdown()
PYPUB

    # 等待评估完成
    echo "  Waiting for evaluation..."
    for i in $(seq 1 90); do
        if grep -q '综合得分' "$LOG_FILE" 2>/dev/null; then
            echo "  Done after ${i}s"
            break
        fi
        sleep 1
    done

    # 等待 capture 完成
    wait $CAPTURE_PID 2>/dev/null || true

    # 提取评估数据
    python3 - "$NAME" "$LOG_FILE" "$PATH_FILE" "$RESULT_DIR" << 'PYEXTRACT'
import sys, re, json, os

name = sys.argv[1]
log_file = sys.argv[2]
path_file = sys.argv[3]
result_dir = sys.argv[4]

with open(log_file) as f:
    text = f.read()

# 提取评估指标
def extract(pat, text, cast=float):
    m = re.search(pat, text)
    if m:
        val = m.group(1)
        return cast(val) if cast != int else int(val)
    return None

eval_data = {
    'scenario': name,
    'coverage_rate': extract(r'覆盖率[:\s]*([\d.]+)%', text),
    'single_score': extract(r'综合得分[:\s]*([\d.]+)', text),
    'uncovered_area': extract(r'未覆盖面积[:\s]*([\d.]+)', text),
    'total_distance': extract(r'路径总长[:\s]*([\d.]+)', text),
    'work_ratio': extract(r'有效工作比[:\s]*([\d.]+)%', text),
    'turn_count': extract(r'转弯次数[:\s]*(\d+)', text, int),
    'overlap_rate': extract(r'重叠率[:\s]*([\d.]+)%', text),
    'planning_time_ms': extract(r'规划耗时[:\s]*([\d.]+)\s*ms', text),
    'coverage_method': extract(r'覆盖率方法[:\s]*(\w+)', text, str),
}

# 提取 net_area
m = re.search(r'目标净面积[:\s]*([\d.]+)', text)
if m: eval_data['net_area'] = float(m.group(1))

# 读取路径数据
path_pts = []
if os.path.exists(path_file):
    with open(path_file) as f:
        path_pts = json.load(f).get('path', [])

# 保存合并数据
merged = {
    'scenario': name,
    'path': path_pts,
    'swaths': [],  # Will be extracted separately if available
    'eval': eval_data,
}

merged_file = f'{result_dir}/{name}_data.json'
with open(merged_file, 'w') as f:
    json.dump(merged, f, indent=2, ensure_ascii=False)

# 打印摘要
cov = eval_data.get('coverage_rate', 0) or 0
score = eval_data.get('single_score', 0) or 0
print(f'  {name}: coverage={cov:.2f}% score={score:.1f} path_pts={len(path_pts)}')
PYEXTRACT

    # 终止 planner
    kill $PLANNER_PID 2>/dev/null || true
    wait $PLANNER_PID 2>/dev/null || true

    echo "  $NAME complete."
done

echo ""
echo "========================================="
echo "  批量测试完成! 生成可视化..."
echo "========================================="

# 渲染所有场景
for entry in "${SCENARIOS[@]}"; do
    NAME="${entry%%:*}"
    YAML="${entry##*:}"
    DATA_FILE="$RESULT_DIR/${NAME}_data.json"
    PNG_FILE="$RESULT_DIR/${NAME}_coverage.png"

    if [ -f "$DATA_FILE" ]; then
        python3 "$WS/render_coverage.py" "$NAME" "$YAML" "$DATA_FILE" "$PNG_FILE" 2>&1
    fi
done

# 生成汇总报告
python3 - "$RESULT_DIR" "${SCENARIOS[@]}" << 'PYREPORT'
import sys, json, os

result_dir = sys.argv[1]

print("")
print("=" * 70)
print("  F2C 6场景覆盖规划 — 批量测试报告")
print("=" * 70)
print("")
print("| 场景 | 面积(m²) | 覆盖率 | 得分 | 未覆盖(m²) | 路径长(m) | 重叠率 | 耗时(ms) |")
print("|:----:|:--------:|:------:|:----:|:---------:|:---------:|:------:|:--------:|")

for name in ['S1','S2','S3','S4','S5','S6']:
    data_file = f'{result_dir}/{name}_data.json'
    if os.path.exists(data_file):
        with open(data_file) as f:
            d = json.load(f)
        e = d.get('eval', {})
        area = e.get('net_area', 0) or 0
        cov = (e.get('coverage_rate', 0) or 0)
        score = e.get('single_score', 0) or 0
        uncov = e.get('uncovered_area', 0) or 0
        dist = e.get('total_distance', 0) or 0
        overlap = e.get('overlap_rate', 0) or 0
        ptime = e.get('planning_time_ms', 0) or 0
        print(f"| {name} | {area:.0f} | {cov:.2f}% | {score:.1f} | {uncov:.2f} | {dist:.1f} | {overlap:.1f}% | {ptime:.0f} |")

print("")
print("## 可视化")
for name in ['S1','S2','S3','S4','S5','S6']:
    png = f'{result_dir}/{name}_coverage.png'
    if os.path.exists(png):
        print(f"### {name}")
        print(f"![]({png})")
        print("")

print(f"")
print(f"报告目录: {result_dir}")
print(f"原始日志: {result_dir}/*_planner.log")
print(f"路径数据: {result_dir}/*_data.json")
PYREPORT

echo ""
echo "Done! Report directory: $RESULT_DIR"
