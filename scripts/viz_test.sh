#!/bin/bash
# 单场景可视化测试 — 验证 C++ Vis JSON 导出
source /opt/ros/humble/setup.bash
source ~/f2c_coverage_planner/install/setup.bash
export LD_LIBRARY_PATH="$HOME/f2c_coverage_planner/install/lib:$HOME/f2c_coverage_planner/src/Fields2Cover/build/_deps/steering_functions-build:$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

rm -f /tmp/f2c_vis_polygon_1.json

ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p robot_width:=0.95 -p coverage_width:=0.45 -p mid_hl_width_ratio:=0.20 \
  -p no_hl_width_ratio:=0.0 -p min_hole_area:=1.0 -p swath_endpoint_shrink_distance:=0.03 \
  -p min_swath_length:=0.5 -p max_diff_curv:=0.3 -p path_resolution:=0.1 \
  -p boundary_type:=closed -p use_optimized_planner:=true \
  -p swath_angle_optimization:=true -p decomposition_enabled:=true \
  -p filter_tiny_cells:=true -p path_simplify_enabled:=true \
  -p path_simplify_tolerance:=0.05 -p turn_planner_type:=direct \
  -p swath_overlap_ratio:=0.03 -p use_sweep_decomp:=true \
  -p merge_angle_threshold:=60.0 -p swath_order_type:=boustrophedon \
  -p eval_enable_report:=true -p eval_use_grid_method:=true \
  -p eval_grid_resolution:=0.1 -p eval_coverage_threshold:=0.995 > /tmp/viz_test.log 2>&1 &

sleep 3

python3 - ~/f2c_coverage_planner/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml << 'PYEOF'
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
rclpy.init()
n = Node("pub_test")
with open(sys.argv[1]) as f:
    a = yaml.safe_load(f)
pp = n.create_publisher(Polygon, "/input_polygon_1", 10)
hp = n.create_publisher(Polygon, "/input_polygon_1_holes", 10)
pm = Polygon()
for p in a["polygon"]:
    pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hm = Polygon()
for hi, h in enumerate(a.get("holes", [])):
    if hi > 0:
        hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in h:
        hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hp.publish(hm)
rclpy.spin_once(n, timeout_sec=0.1)
time.sleep(0.5)
pp.publish(pm)
time.sleep(1.0)
rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(60):
    rclpy.spin_once(n, timeout_sec=0.5)
    time.sleep(0.5)
n.destroy_node()
rclpy.shutdown()
PYEOF

sleep 3

echo "=== Vis JSON ==="
ls -la /tmp/f2c_vis_polygon_1.json 2>/dev/null && head -3 /tmp/f2c_vis_polygon_1.json && echo "..." && wc -c /tmp/f2c_vis_polygon_1.json

echo "=== Eval count ==="
grep -c "综合得分" /tmp/viz_test.log 2>/dev/null || echo "0"

echo "=== Vis JSON count ==="
grep -c "Vis JSON" /tmp/viz_test.log 2>/dev/null || echo "0"

echo "=== Last eval ==="
grep "综合得分\|覆盖率:" /tmp/viz_test.log 2>/dev/null | tail -5

echo "=== Rendered PNG ==="
if [ -f /tmp/f2c_vis_polygon_1.json ]; then
    python3 ~/f2c_coverage_planner/scripts/render_coverage.py "S1_test" \
        ~/f2c_coverage_planner/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml \
        /tmp/f2c_vis_polygon_1.json \
        /tmp/S1_test.png
    ls -la /tmp/S1_test.png
fi

# cleanup
pkill -f polygon_planner_node 2>/dev/null || true
