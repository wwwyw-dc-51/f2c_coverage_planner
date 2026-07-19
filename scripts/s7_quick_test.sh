#!/bin/bash
# S7 工厂车间快速测试脚本
# 用法：在 VM 上运行
#   bash scripts/s7_quick_test.sh           # 单次测试
#   bash scripts/s7_quick_test.sh --loop N  # 循环 N 次

set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="S7"
YAML="$WS/src/yingshi_robot/test_polygons/S7_S7_factory_workshop.yaml"
LOG="/tmp/s7_test_$(date +%H%M%S).log"
LOOPS="${2:-1}"

if [ ! -f "$YAML" ]; then
    echo "❌ S7 YAML 不存在: $YAML"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export LD_LIBRARY_PATH="$WS/install/lib:$WS/install/fields2cover/lib:$WS/src/Fields2Cover/third_party/ortools-src/lib:$LD_LIBRARY_PATH"

for ((run=1; run<=LOOPS; run++)); do
    LOG="/tmp/s7_test_$(date +%H%M%S).log"
    echo "=== [S7] Run $run/$LOOPS $(date +%H:%M:%S) ==="

    ros2 run yingshi_robot polygon_planner_node --ros-args \
        -p use_planner_core:=true \
        -p robot_width:=0.75 -p coverage_width:=0.90 \
        -p mid_hl_width_ratio:=0.20 -p no_hl_width_ratio:=0.0 \
        -p use_sweep_decomp:=true -p merge_angle_threshold:=60.0 \
        -p swath_order_type:=boustrophedon \
        -p boundary_type:=closed -p turn_planner_type:=direct \
        -p use_optimized_planner:=true -p decomposition_enabled:=true \
        -p path_simplify_enabled:=true -p swath_angle_optimization:=true \
        -p filter_tiny_cells:=true -p swath_endpoint_shrink_distance:=0.03 \
        -p min_swath_length:=0.5 -p path_simplify_tolerance:=0.05 \
        -p swath_overlap_ratio:=0.03 -p eval_enable_report:=true \
        -p eval_use_grid_method:=true -p eval_grid_resolution:=0.1 \
        > "$LOG" 2>&1 &
    PID=$!
    sleep 4

    python3 - "$YAML" << 'PYEOF'
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
rclpy.init()
n = Node('qt')
with open(sys.argv[1]) as f:
    a = yaml.safe_load(f)
pp = n.create_publisher(Polygon, '/input_polygon_1', 10)
hp = n.create_publisher(Polygon, '/input_polygon_1_holes', 10)
pm = Polygon()
for p in a['polygon']:
    pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hm = Polygon()
for hi, h in enumerate(a.get('holes', [])):
    if hi > 0:
        hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in h:
        hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
for _ in range(3):
    hp.publish(hm); rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.2)
    pp.publish(pm); time.sleep(2.0); rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(90):
    rclpy.spin_once(n, timeout_sec=0.5); time.sleep(0.5)
n.destroy_node(); rclpy.shutdown()
PYEOF
    wait $PID 2>/dev/null || true

    echo "--- S7 Results ---"
    grep -E '覆盖率|综合得分|重叠率|路径总长|未覆盖面积|有效工作比|hole.aware|Gap classif|cells.*decomposed' "$LOG" | head -20
    echo "LOG=$LOG"
    echo ""
done
