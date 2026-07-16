#!/usr/bin/env bash
# 快速单场景测试 — 无 RViz，仅控制台输出关键指标
# 用法: ./quick_test.sh [yaml文件路径]
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
AREA_FILE="${1:-$WS/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml}"
SCENARIO=$(basename "$AREA_FILE" .yaml)

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

LOG="/tmp/quick_test_$(date +%H%M%S).log"

echo "=== $SCENARIO ==="

# Start planner
ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p robot_width:=0.95 -p coverage_width:=0.45 \
  -p mid_hl_width_ratio:=0.20 -p no_hl_width_ratio:=0.0 \
  -p use_sweep_decomp:=true -p sweep_merge_angle_threshold:=60.0 \
  -p boundary_type:=closed -p turn_planner_type:=direct \
  -p use_optimized_planner:=true -p path_simplify_enabled:=true \
  -p swath_angle_optimization:=true -p filter_tiny_cells:=true \
  -p swath_endpoint_shrink_distance:=0.03 -p min_swath_length:=0.5 \
  -p path_simplify_tolerance:=0.05 -p swath_overlap_ratio:=0.03 \
  -p eval_enable_report:=true -p eval_use_grid_method:=true \
  -p eval_grid_resolution:=0.1 -p decomposition_enabled:=true \
  > "$LOG" 2>&1 &

PLANNER_PID=$!
sleep 3

# Publish polygon
python3 - "$AREA_FILE" << PYEOF
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

rclpy.init()
n = Node("qt")
with open(sys.argv[1]) as f:
    a = yaml.safe_load(f)
pp = n.create_publisher(Polygon, "/input_polygon_1", 10)
hp = n.create_publisher(Polygon, "/input_polygon_1_holes", 10)
pm = Polygon()
for p in a["polygon"]:
    pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hm = Polygon()
for hi, h in enumerate(a.get("holes", [])):
    if hi > 0: hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in h: hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
for _ in range(3):
    hp.publish(hm); rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.2)
    pp.publish(pm); time.sleep(2.0); rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(90):
    rclpy.spin_once(n, timeout_sec=0.5); time.sleep(0.5)
n.destroy_node(); rclpy.shutdown()
PYEOF

wait $PLANNER_PID 2>/dev/null

echo ""
grep -E "Simplified path.*crossing|CROSSES|覆盖率:|综合得分|Post-planPath" "$LOG" | head -5
grep -A2 "综合评分" "$LOG" | head -4
echo "Full log: $LOG"
