#!/usr/bin/env bash
# F2C 全覆盖规划 — 7场景全量基准测试（custom vs snake）
# v2: 修复 wait 死等 bug，改用 grep 检测完成 + kill
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
TIMESTAMP=$(date +%m%d_%H%M)
RESULT_DIR="$WS/test_results/full_bench_${TIMESTAMP}"
mkdir -p "$RESULT_DIR"

set +u; source /opt/ros/humble/setup.bash; set -u
set +u; source "$WS/install/setup.bash"; set -u
export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

declare -A YAMLS=(
    ["notched"]="$WS/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"
    ["S1"]="$WS/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml"
    ["S2"]="$WS/src/yingshi_robot/test_polygons/S2_S2_L_shaped.yaml"
    ["S3"]="$WS/src/yingshi_robot/test_polygons/S3_S3_with_holes.yaml"
    ["S4"]="$WS/src/yingshi_robot/test_polygons/S4_S4_narrow_corridor.yaml"
    ["S5"]="$WS/src/yingshi_robot/test_polygons/S5_S5_irregular.yaml"
    ["S6"]="$WS/src/yingshi_robot/test_polygons/S6_S6_multi_region.yaml"
)

MODES=("boustrophedon" "snake")
MODE_LABELS=("custom" "snake")

echo "========================================="
echo "  F2C Full Benchmark v2: 7 scenarios x 2 modes"
echo "  Output: $RESULT_DIR"
echo "========================================="

SUMMARY_JSON="$RESULT_DIR/summary.json"
echo "{" > "$SUMMARY_JSON"
echo '  "timestamp": "'$(date -Iseconds)'",' >> "$SUMMARY_JSON"
echo '  "params": {"robot_width":0.95,"coverage_width":0.45,"sweep":true,"RDP":0.05,"turn":"direct"},' >> "$SUMMARY_JSON"
echo '  "results": [' >> "$SUMMARY_JSON"

FIRST_RESULT=true

for NAME in notched S1 S2 S3 S4 S5 S6; do
    YAML="${YAMLS[$NAME]}"
    BOUNDARY=$(python3 -c "import yaml; print(yaml.safe_load(open('$YAML'))['boundary_type'])" 2>/dev/null || echo "closed")

    for mi in 0 1; do
        MODE="${MODES[$mi]}"
        LABEL="${MODE_LABELS[$mi]}"

        echo ""
        echo "=== [$NAME] $LABEL mode ==="

        LOG="$RESULT_DIR/${NAME}_${LABEL}.log"

        ros2 run yingshi_robot polygon_planner_node --ros-args \
          -p robot_width:=0.95 \
          -p coverage_width:=0.45 \
          -p mid_hl_width_ratio:=0.20 \
          -p no_hl_width_ratio:=0.0 \
          -p min_hole_area:=1.0 \
          -p swath_endpoint_shrink_distance:=0.03 \
          -p min_swath_length:=0.5 \
          -p max_diff_curv:=0.3 \
          -p path_resolution:=0.1 \
          -p boundary_type:=$BOUNDARY \
          -p use_optimized_planner:=true \
          -p swath_angle_optimization:=true \
          -p decomposition_enabled:=true \
          -p filter_tiny_cells:=true \
          -p path_simplify_enabled:=true \
          -p path_simplify_tolerance:=0.05 \
          -p turn_planner_type:=direct \
          -p swath_overlap_ratio:=0.03 \
          -p use_sweep_decomp:=true \
          -p merge_angle_threshold:=60.0 \
          -p swath_order_type:=$MODE \
          -p eval_enable_report:=true \
          -p eval_use_grid_method:=true \
          -p eval_grid_resolution:=0.1 \
          -p eval_coverage_threshold:=0.995 > "$LOG" 2>&1 &

        PID=$!
        sleep 3

        # 发布多边形
        python3 - "$YAML" > "$RESULT_DIR/${NAME}_${LABEL}_pub.log" 2>&1 << PYEOF
import sys, time, yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

rclpy.init()
n = Node("pub_bench")
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
for _ in range(3):
    hp.publish(hm); rclpy.spin_once(n, timeout_sec=0.1); time.sleep(0.2)
    pp.publish(pm); time.sleep(2.0); rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(90):
    rclpy.spin_once(n, timeout_sec=0.5); time.sleep(0.5)
n.destroy_node(); rclpy.shutdown()
PYEOF

        # 等评估报告出来（最长90秒），然后杀 planner
        for i in $(seq 1 45); do
            if grep -q "综合得分" "$LOG" 2>/dev/null; then
                sleep 1
                break
            fi
            sleep 2
        done
        kill $PID 2>/dev/null || true
        wait $PID 2>/dev/null || true
        sleep 1

        # 提取指标
        COV=$(grep -oP "覆盖率[:\s]+\K[0-9.]+" "$LOG" | tail -1 || echo "0")
        SCORE=$(grep -oP "综合得分[:\s]+\K[0-9.]+" "$LOG" | tail -1 || echo "0")
        TURNS=$(grep -oP "转弯次数[:\s]+\K[0-9]+" "$LOG" | tail -1 || echo "0")
        OVERLAP=$(grep -oP "重叠率[:\s]+\K[0-9.]+" "$LOG" | tail -1 || echo "0")
        DIST=$(grep -oP "路径总长[:\s]+\K[0-9.]+" "$LOG" | tail -1 || echo "0")
        OUTBOUND=$(grep -c "crosses hole\|HOLE" "$LOG" || echo "0")

        echo "  Coverage=$COV% Score=$SCORE Turns=$TURNS Overlap=$OVERLAP% Dist=${DIST}m"

        if [ "$FIRST_RESULT" = true ]; then FIRST_RESULT=false; else echo "," >> "$SUMMARY_JSON"; fi
        echo -n "    {\"scenario\":\"$NAME\",\"mode\":\"$LABEL\",\"coverage\":$COV,\"score\":$SCORE,\"turns\":$TURNS,\"overlap\":$OVERLAP,\"distance\":$DIST,\"outbound_hits\":$OUTBOUND}" >> "$SUMMARY_JSON"
    done
done

echo "" >> "$SUMMARY_JSON"
echo "  ]" >> "$SUMMARY_JSON"
echo "}" >> "$SUMMARY_JSON"

echo ""
echo "========================================="
echo "  Benchmark complete!"
echo "  Results: $SUMMARY_JSON"
echo "========================================="
cat "$SUMMARY_JSON"
