#!/bin/bash
# VM 端测试脚本：启动 planner + 发送多边形 + 等待结果
# 由 sync_and_build.sh 远程调用
set -e

SCENARIO="${1:-notched}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 场景→YAML 映射
declare -A SCENARIOS=(
    ["notched"]="${WS}/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"
    ["S1"]="${WS}/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml"
    ["S2"]="${WS}/src/yingshi_robot/test_polygons/S2_S2_L_shaped.yaml"
    ["S3"]="${WS}/src/yingshi_robot/test_polygons/S3_S3_with_holes.yaml"
    ["S4"]="${WS}/src/yingshi_robot/test_polygons/S4_S4_narrow_corridor.yaml"
    ["S5"]="${WS}/src/yingshi_robot/test_polygons/S5_S5_irregular.yaml"
    ["S6"]="${WS}/src/yingshi_robot/test_polygons/S6_S6_multi_region.yaml"
)

YAML="${SCENARIOS[$SCENARIO]:-${SCENARIOS[notched]}}"

source /opt/ros/humble/setup.bash
source ${WS}/install/setup.bash
export LD_LIBRARY_PATH=${WS}/install/lib:${WS}/install/fields2cover/lib:${WS}/src/Fields2Cover/third_party/ortools-src/lib:$LD_LIBRARY_PATH

LOG="/tmp/f2c_test_$(date +%H%M%S).log"

echo "Starting planner..."
ros2 run yingshi_robot polygon_planner_node --ros-args \
    -p robot_width:=0.95 -p coverage_width:=0.45 \
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
    > $LOG 2>&1 &

PLANNER_PID=$!
sleep 4

echo "Sending polygon..."
python3 ${WS}/scripts/send_test_polygon.py "$YAML"

wait $PLANNER_PID 2>/dev/null || true

echo ""
echo "=== RESULTS ==="
grep -E '覆盖率|综合得分|转弯次数|重叠率|路径总长|未覆盖面积|有效工作比|hole.aware|RemArea fill|greedy|Cell traversal' $LOG | head -20
echo "LOG=$LOG"
