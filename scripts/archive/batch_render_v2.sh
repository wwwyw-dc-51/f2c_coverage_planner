#!/usr/bin/env bash
# F2C 路径可视化 v2 — 利用 C++ 自动导出的 vis JSON，不需 ROS 订阅
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
RESULT_DIR="$WS/test_results/viz_clean_$(date +%m%d_%H%M)"
mkdir -p "$RESULT_DIR"

set +u; source /opt/ros/humble/setup.bash; set -u
set +u; source "$WS/install/setup.bash"; set -u
export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

declare -A YAMLS=(
    ["S1"]="$WS/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml"
    ["S2"]="$WS/src/yingshi_robot/test_polygons/S2_S2_L_shaped.yaml"
    ["S3"]="$WS/src/yingshi_robot/test_polygons/S3_S3_with_holes.yaml"
    ["S4"]="$WS/src/yingshi_robot/test_polygons/S4_S4_narrow_corridor.yaml"
    ["S5"]="$WS/src/yingshi_robot/test_polygons/S5_S5_irregular.yaml"
    ["S6"]="$WS/src/yingshi_robot/test_polygons/S6_S6_multi_region.yaml"
    ["notched"]="$WS/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"
)

MODES=("boustrophedon" "snake")
MODE_LABELS=("custom" "snake")

echo "========================================="
echo "  F2C Viz v2: C++ auto-export JSON"
echo "  Output: $RESULT_DIR"
echo "========================================="

for NAME in S1 S2 S3 S4 S5 S6 notched; do
    YAML="${YAMLS[$NAME]}"
    BOUNDARY=$(python3 -c "import yaml; print(yaml.safe_load(open('$YAML'))['boundary_type'])" 2>/dev/null || echo "closed")

    for mi in 0 1; do
        MODE="${MODES[$mi]}"
        LABEL="${MODE_LABELS[$mi]}"

        echo ""
        echo "=== [$NAME] $LABEL mode ==="

        LOG="$RESULT_DIR/${NAME}_${LABEL}.log"
        VIS_JSON="/tmp/f2c_vis_polygon_1.json"
        rm -f "$VIS_JSON"  # 清除旧文件

        # 启动 planner（新编译版，会自动导出 vis JSON）
        ros2 run yingshi_robot polygon_planner_node --ros-args \
          -p robot_width:=0.95 -p coverage_width:=0.45 -p mid_hl_width_ratio:=0.20 \
          -p no_hl_width_ratio:=0.0 -p min_hole_area:=1.0 -p swath_endpoint_shrink_distance:=0.03 \
          -p min_swath_length:=0.5 -p max_diff_curv:=0.3 -p path_resolution:=0.1 \
          -p boundary_type:=$BOUNDARY -p use_optimized_planner:=true \
          -p swath_angle_optimization:=true -p decomposition_enabled:=true \
          -p filter_tiny_cells:=true -p path_simplify_enabled:=true \
          -p path_simplify_tolerance:=0.05 -p turn_planner_type:=direct \
          -p swath_overlap_ratio:=0.03 -p use_sweep_decomp:=true \
          -p merge_angle_threshold:=60.0 -p swath_order_type:=$MODE \
          -p eval_enable_report:=true -p eval_use_grid_method:=true \
          -p eval_grid_resolution:=0.1 -p eval_coverage_threshold:=0.995 > "$LOG" 2>&1 &

        PID=$!
        sleep 3

        # 发布多边形 — 只发一次！
        python3 - "$YAML" > "$RESULT_DIR/${NAME}_${LABEL}_pub.log" 2>&1 << 'PYEOF'
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

rclpy.init()
n = Node("pub_once")
with open(sys.argv[1]) as f: a = yaml.safe_load(f)
pp = n.create_publisher(Polygon, "/input_polygon_1", 10)
hp = n.create_publisher(Polygon, "/input_polygon_1_holes", 10)

pm = Polygon()
for p in a["polygon"]:
    pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))
hm = Polygon()
for hi, h in enumerate(a.get("holes", [])):
    if hi > 0: hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for p in h: hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

# 只发一次！
hp.publish(hm); rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.5)
pp.publish(pm); time.sleep(1.0); rclpy.spin_once(n, timeout_sec=0.1)

# 等评估完成
for _ in range(120):
    rclpy.spin_once(n, timeout_sec=0.5); time.sleep(0.5)
n.destroy_node(); rclpy.shutdown()
PYEOF

        # 等评估完成 + vis JSON 写出
        for i in $(seq 1 50); do
            if grep -q "Vis JSON saved" "$LOG" 2>/dev/null; then sleep 1; break; fi
            if grep -q "综合得分" "$LOG" 2>/dev/null; then sleep 2; break; fi
            sleep 2
        done
        kill $PID 2>/dev/null || true
        wait $PID 2>/dev/null || true

        # 用 C++ 导出的 vis JSON 渲染
        PNG="$RESULT_DIR/${NAME}_${LABEL}.png"
        if [ -f "$VIS_JSON" ]; then
            python3 "$WS/scripts/render_coverage.py" "${NAME}_${LABEL}" "$YAML" "$VIS_JSON" "$PNG" 2>&1 || true
            echo "  ✓ $PNG"
        else
            echo "  ✗ No vis JSON — check $LOG"
        fi
    done
done

echo ""
echo "========================================="
echo "  Done!"
echo "========================================="
ls -la "$RESULT_DIR"/*.png 2>/dev/null | wc -l
echo "PNGs in $RESULT_DIR"
