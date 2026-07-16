#!/usr/bin/env bash
# 批量路径可视化：7场景 × 2模式 = 14张PNG
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
RESULT_DIR="$WS/test_results/viz_$(date +%m%d_%H%M)"
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
echo "  Batch Viz: 7 scenarios x 2 modes"
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

        LOG_FILE="$RESULT_DIR/${NAME}_${LABEL}_planner.log"
        DATA_JSON="$RESULT_DIR/${NAME}_${LABEL}_data.json"
        PNG_OUT="$RESULT_DIR/${NAME}_${LABEL}.png"

        # 启动 planner
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
          -p eval_coverage_threshold:=0.995 > "$LOG_FILE" 2>&1 &

        PID=$!
        sleep 3

        # 采集路径数据
        python3 - "$YAML" "$RESULT_DIR" "$NAME" "$LABEL" > "$RESULT_DIR/${NAME}_${LABEL}_pub.log" 2>&1 << 'PYEOF'
import sys, os, time, yaml, json, re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

class QuickCapture(Node):
    def __init__(self, yaml_path, out_dir, name, label):
        super().__init__('qc_' + name + '_' + label)
        self.path_pts = []
        self.path_done = False
        self.out_dir = out_dir
        self.name = name
        self.label = label
        self.yaml_path = yaml_path
        self.sub = self.create_subscription(Path, '/planned2_path_1', self.on_path, 10)
        self.pp = self.create_publisher(Polygon, '/input_polygon_1', 10)
        self.hp = self.create_publisher(Polygon, '/input_polygon_1_holes', 10)

    def on_path(self, msg):
        if len(msg.poses) > 0 and not self.path_done:
            for p in msg.poses:
                self.path_pts.append({'x': p.pose.position.x, 'y': p.pose.position.y})
            self.path_done = True

    def run(self):
        with open(self.yaml_path) as f:
            a = yaml.safe_load(f)
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
            self.hp.publish(hm); rclpy.spin_once(self, timeout_sec=0.1); time.sleep(0.2)
            self.pp.publish(pm); time.sleep(2.0); rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(90):
            rclpy.spin_once(self, timeout_sec=0.5); time.sleep(0.5)
            if self.path_done:
                break

        log_file = f'{self.out_dir}/{self.name}_{self.label}_planner.log'
        cov = 0.0; score = 0.0
        if os.path.exists(log_file):
            with open(log_file) as f:
                txt = f.read()
            cm = re.search(r'覆盖率[:\s]+([\d.]+)%', txt)
            sm = re.search(r'综合得分[:\s]+([\d.]+)', txt)
            if cm: cov = float(cm.group(1))
            if sm: score = float(sm.group(1))

        data = {
            'path': self.path_pts,
            'eval': {'coverage_rate': cov / 100.0, 'single_score': score}
        }
        json_path = f'{self.out_dir}/{self.name}_{self.label}_data.json'
        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'Captured: {len(self.path_pts)} path pts, cov={cov}%, score={score}')

        self.destroy_node()

rclpy.init()
qc = QuickCapture(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
qc.run()
rclpy.shutdown()
PYEOF

        # 等评估完成
        for i in $(seq 1 45); do
            if grep -q "综合得分" "$LOG_FILE" 2>/dev/null; then sleep 1; break; fi
            sleep 2
        done
        kill $PID 2>/dev/null || true
        wait $PID 2>/dev/null || true

        # 渲染 PNG
        if [ -f "$DATA_JSON" ]; then
            python3 "$WS/scripts/render_coverage.py" "${NAME}_${LABEL}" "$YAML" "$DATA_JSON" "$PNG_OUT" 2>&1 || true
            echo "  PNG: $PNG_OUT"
        else
            echo "  WARNING: No data JSON, skipping"
        fi
    done
done

echo ""
echo "========================================="
echo "  Done! PNGs in $RESULT_DIR"
echo "========================================="
ls -la "$RESULT_DIR"/*.png 2>/dev/null || echo "No PNGs found"
