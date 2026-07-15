#!/usr/bin/env bash
# Quick S3 sweep decomposition test
set -euo pipefail
WS="$HOME/f2c_coverage_planner"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

LOG=/tmp/s3_sweep_$$.log

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
  -p eval_use_grid_method:=false \
  -p eval_coverage_threshold:=0.995 \
  -p use_sweep_decomp:=true \
  > "$LOG" 2>&1 &

PLANNER_PID=$!
sleep 3

# Publish polygon (needs ROS2 sourced)
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
python3 "$WS/pub_s3.py" >> "$LOG" 2>&1 || true

# Wait for eval
for i in $(seq 1 30); do
  if grep -q '综合得分' "$LOG" 2>/dev/null; then break; fi
  sleep 1
done

kill $PLANNER_PID 2>/dev/null; wait $PLANNER_PID 2>/dev/null || true

echo "=== Decomposition ==="
grep -E '(Decompose|sweep|Decomposed|cell_count|Adaptive)' "$LOG" | head -10
echo ""
echo "=== Results ==="
grep -E '(覆盖率|综合得分|swath_count|turn_count|重叠率)' "$LOG" | head -10
