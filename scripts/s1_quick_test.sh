#!/usr/bin/env bash
set -eo pipefail

unset PATH
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin
export AMENT_PREFIX_PATH=/opt/ros/humble:/home/dc/f2c_coverage_planner/install
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/home/dc/f2c_coverage_planner/install/lib
export DISPLAY=:0

source /opt/ros/humble/setup.bash
source /home/dc/f2c_coverage_planner/install/setup.bash
cd /home/dc/f2c_coverage_planner

pkill -f polygon_planner_node 2>/dev/null || true
sleep 1

WS=/home/dc/f2c_coverage_planner
YAML=$WS/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml
LOG=/tmp/s1_test.log
rm -f /tmp/f2c_vis_polygon_1.json /tmp/f2c_grid_polygon_1.json "$LOG"

echo '=== Starting planner ==='
ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p use_planner_core:=true \
  -p robot_width:=0.75 \
  -p coverage_width:=0.90 \
  -p boundary_type:=closed \
  > "$LOG" 2>&1 &
PLANNER_PID=$!
sleep 3

echo '=== Publishing polygon ==='
python3 -c "
import yaml, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
rclpy.init()
node = Node('test_pub')
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)
with open('$YAML') as f:
    area = yaml.safe_load(f)
poly = Polygon()
for x, y in area['polygon']:
    p = Point32(); p.x = float(x); p.y = float(y)
    poly.points.append(p)
# 发送主多边形和空 holes
poly_pub.publish(poly)
holes_pub.publish(Polygon())  # 空 holes
print('Published polygon + empty holes')
time.sleep(20)
rclpy.shutdown()
"

echo '=== Planner output ==='
cat "$LOG" | tail -30

echo '=== Coverage ==='
if [ -f /tmp/f2c_vis_polygon_1.json ]; then
    python3 -c "import json; d=json.load(open('/tmp/f2c_vis_polygon_1.json')); print(d.get('coverage_rate', 'N/A'))"
fi

wait $PLANNER_PID 2>/dev/null || true
echo "exit: $?"
