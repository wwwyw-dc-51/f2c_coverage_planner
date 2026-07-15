#!/usr/bin/env bash
# F2C: 自定义算法 vs Snake 模式可视化对比 (S3/S4/S6)
set -euo pipefail

WS="$HOME/f2c_coverage_planner"
RESULT_DIR="$WS/test_results/compare_snake_$(date +%m%d_%H%M)"
RENDER_SCRIPT="$WS/scripts/render_coverage.py"
mkdir -p "$RESULT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export LD_LIBRARY_PATH="$WS/install/lib:$WS/src/Fields2Cover/build/_deps/steering_functions-build:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

TEST_POLYGONS="$WS/src/yingshi_robot/test_polygons"
declare -A SCENARIOS=(
    ["S3"]="$TEST_POLYGONS/S3_S3_with_holes.yaml"
    ["S4"]="$TEST_POLYGONS/S4_S4_narrow_corridor.yaml"
    ["S6"]="$TEST_POLYGONS/S6_S6_multi_region.yaml"
)

cleanup_ros() {
    pkill -f "polygon_planner_node" 2>/dev/null || true
    pkill -f "test_publisher" 2>/dev/null || true
    sleep 2
}

run_single_test() {
    local NAME="$1"
    local YAML="$2"
    local MODE="$3"   # custom 或 snake
    local MODE_LABEL="$4"
    local TAG="${NAME}_${MODE}"

    echo "  [$TAG] Starting ($MODE_LABEL)..."
    cleanup_ros

    local LOG_FILE="$RESULT_DIR/${TAG}.log"
    local ORDER_PARAM="none"
    [[ "$MODE" == "snake" ]] && ORDER_PARAM="snake"

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
        -p use_sweep_decomp:=true \
        -p sweep_merge_angle_threshold:=60.0 \
        -p path_simplify_enabled:=true \
        -p path_simplify_tolerance:=0.05 \
        -p path_simplify_turn_threshold:=0.15 \
        -p turn_planner_type:=direct \
        -p swath_overlap_ratio:=0.03 \
        -p swath_order_type:="$ORDER_PARAM" \
        -p eval_enable_report:=true \
        -p eval_use_grid_method:=true \
        -p eval_grid_resolution:=0.1 \
        -p eval_coverage_threshold:=0.995 \
        > "$LOG_FILE" 2>&1 &
    local PLANNER_PID=$!

    sleep 3
    if ! kill -0 $PLANNER_PID 2>/dev/null; then
        echo "  [$TAG] ERROR: planner crashed"
        tail -5 "$LOG_FILE"
        return 1
    fi

    # Python 发布者 + 路径捕获
    python3 - "$NAME" "$YAML" "$LOG_FILE" "$RESULT_DIR" "$TAG" << 'PYTEST'
import sys, time, yaml, json, os, re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

HOLE_SEP = 1e10
name = sys.argv[1]
yaml_path = sys.argv[2]
log_file = sys.argv[3]
result_dir = sys.argv[4]
tag = sys.argv[5]

rclpy.init()
node = rclpy.create_node(f'test_{tag}')

path_data = []
plan_received = False

def on_path(msg):
    global path_data, plan_received
    if not plan_received and len(msg.poses) > 0:
        path_data = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
        plan_received = True

node.create_subscription(Path, '/planned2_path_1', on_path, 10)
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

with open(yaml_path) as f:
    area = yaml.safe_load(f)

poly_msg = Polygon()
for p in area['polygon']:
    poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

holes_msg = Polygon()
for hi, hole in enumerate(area.get('holes', [])):
    if hi > 0:
        holes_msg.points.append(Point32(x=HOLE_SEP, y=0.0, z=0.0))
    for p in hole:
        holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

time.sleep(1.0)

for attempt in range(5):
    holes_pub.publish(holes_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.3)
    poly_pub.publish(poly_msg)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.5)
        if plan_received:
            break
    if plan_received:
        print(f'  Plan received (attempt {attempt+1})')
        break
    print(f'  Retry {attempt+1}...')

# 等待评估完成
for i in range(120):
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    if os.path.exists(log_file):
        with open(log_file) as f:
            if '综合得分' in f.read():
                print(f'  Eval complete after {i+1}s')
                break

# 解析结果
result = {}
with open(log_file) as f:
    text = f.read()

def extract(pat, text, cast=float):
    m = re.search(pat, text)
    return cast(m.group(1)) if m else None

result = {
    'scenario': tag,
    'coverage_rate': extract(r'覆盖率[:\s]*([\d.]+)%', text),
    'single_score': extract(r'综合得分[:\s]*([\d.]+)', text),
    'total_distance': extract(r'路径总长[:\s]*([\d.]+)', text),
    'turn_count': extract(r'转弯次数[:\s]*(\d+)', text, int),
    'overlap_rate': extract(r'重叠率[:\s]*([\d.]+)%', text),
    'work_ratio': extract(r'有效工作比[:\s]*([\d.]+)%', text),
}
cov = result.get('coverage_rate', 0) or 0
score = result.get('single_score', 0) or 0
turns = result.get('turn_count', 0) or 0
print(f'  [{tag}] cov={cov:.2f}% score={score:.1f} turns={turns}')

merged = {'scenario': tag, 'path': path_data, 'swaths': [], 'eval': result}
data_file = f'{result_dir}/{tag}_data.json'
with open(data_file, 'w') as f:
    json.dump(merged, f, indent=2, ensure_ascii=False)

# 保存 C++ 网格数据（避免渲染时重算）
import shutil
grid_src = '/tmp/f2c_grid_polygon_1.json'
grid_dst = f'{result_dir}/{tag}_grid.json'
if os.path.exists(grid_src):
    shutil.copy(grid_src, grid_dst)
    print(f'  Grid data saved: {grid_dst}')

node.destroy_node()
rclpy.shutdown()
PYTEST

    sleep 1
    kill $PLANNER_PID 2>/dev/null || true
    wait $PLANNER_PID 2>/dev/null || true
    echo "  [$TAG] Done"
}

echo "========================================="
echo "  F2C 自定义 vs Snake 可视化对比"
echo "  场景: S3 S4 S6"
echo "  输出: $RESULT_DIR"
echo "========================================="

for NAME in S3 S4 S6; do
    YAML="${SCENARIOS[$NAME]}"
    echo ""
    echo "=== $NAME ==="

    # 先跑自定义算法
    run_single_test "$NAME" "$YAML" "custom" "自定义绕洞"
    # 再跑 Snake 模式
    run_single_test "$NAME" "$YAML" "snake" "Snake排序"
done

echo ""
echo "========================================="
echo "  生成可视化对比图..."
echo "========================================="

# 渲染对比图
python3 - "$RESULT_DIR" "${SCENARIOS[S3]}" "${SCENARIOS[S4]}" "${SCENARIOS[S6]}" "$RENDER_SCRIPT" << 'PYRENDER'
import sys, json, os, yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPolygon
from matplotlib.path import Path as MPath
import numpy as np

rd = sys.argv[1]
yaml_s3 = sys.argv[2]
yaml_s4 = sys.argv[3]
yaml_s6 = sys.argv[4]
render_script = sys.argv[5]

yamls = {'S3': yaml_s3, 'S4': yaml_s4, 'S6': yaml_s6}
names = ['S3', 'S4', 'S6']
modes = ['custom', 'snake']
mode_labels = {'custom': 'Our Custom Algorithm', 'snake': 'F2C Snake Ordering'}

def load_data(scenario, mode):
    tag = f'{scenario}_{mode}'
    df = f'{rd}/{tag}_data.json'
    if not os.path.exists(df):
        print(f'  WARNING: {df} not found')
        return None
    with open(df) as f:
        return json.load(f)

# 为每个场景生成一张对比图(左自定义右Snake，上下行路径+覆盖)
for name in names:
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    with open(yamls[name]) as f:
        area = yaml.safe_load(f)
    outer = [(p[0], p[1]) for p in area['polygon']]
    holes = [[(p[0], p[1]) for p in h] for h in area.get('holes', [])]

    poly_path = MPath(outer)
    hole_paths = [MPath(h) for h in holes]

    for col, mode in enumerate(modes):
        data = load_data(name, mode)
        if data is None:
            continue

        path_pts = data.get('path', [])
        ev = data.get('eval', {})
        cov_rate = (ev.get('coverage_rate', 0) or 0)
        score = ev.get('single_score', 0) or 0
        turns = ev.get('turn_count', 0) or 0

        # 上面: 路径图
        ax_path = axes[0, col]
        ax_path.set_title(f'{mode_labels[mode]}\nPath (turns={turns})', fontsize=11)
        ax_path.add_patch(MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=1.5))
        for h in holes:
            ax_path.add_patch(MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1, alpha=0.7))
        if path_pts:
            px = [p['x'] for p in path_pts]
            py = [p['y'] for p in path_pts]
            ax_path.plot(px, py, color='darkorange', linewidth=0.6, alpha=0.9)
        ax_path.set_aspect('equal')
        ax_path.set_xlabel('X (m)'); ax_path.set_ylabel('Y (m)')

        # 下面: 覆盖热力图
        ax_cov = axes[1, col]
        ax_cov.set_title(f'Coverage: {cov_rate:.1f}% | Score: {score:.1f}', fontsize=11)
        ax_cov.add_patch(MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=1.5))
        for h in holes:
            ax_cov.add_patch(MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1, alpha=0.7))

        # 覆盖热力图：优先使用 C++ 网格数据，避免 Python 重算
        covered_x, covered_y = [], []
        uncovered_x, uncovered_y = [], []
        grid_file = f'{rd}/{name}_{mode}_grid.json'
        if os.path.exists(grid_file):
            with open(grid_file) as f:
                gd = json.load(f)
            covered_x = [p[0] for p in gd.get('covered', [])]
            covered_y = [p[1] for p in gd.get('covered', [])]
            uncovered_x = [p[0] for p in gd.get('uncovered', [])]
            uncovered_y = [p[1] for p in gd.get('uncovered', [])]
            print(f'  [{name}_{mode}] Using C++ grid: {len(covered_x)} covered + {len(uncovered_x)} uncovered')
        else:
            # 回退：网格采样
            res = 0.10
            cov_width = 0.45
            half_w = cov_width / 2.0
            ox, oy = zip(*outer)
            min_x, max_x = min(ox), max(ox)
            min_y, max_y = min(oy), max(oy)

            def in_target(px, py):
                if not poly_path.contains_point((px, py)):
                    return False
                for hp in hole_paths:
                    if hp.contains_point((px, py)):
                        return False
                return True

            if path_pts:
                segs = np.array([(path_pts[i]['x'], path_pts[i]['y'],
                                  path_pts[i+1]['x'], path_pts[i+1]['y'])
                                 for i in range(len(path_pts)-1)])
            else:
                segs = np.zeros((0, 4))

            def is_covered(px, py):
                if len(segs) == 0:
                    return False
                x1, y1, x2, y2 = segs[:,0], segs[:,1], segs[:,2], segs[:,3]
                dx = x2 - x1; dy = y2 - y1
                len2 = dx*dx + dy*dy
                mask = len2 > 1e-12
                t = np.zeros_like(len2)
                t[mask] = np.clip(((px-x1[mask])*dx[mask] + (py-y1[mask])*dy[mask]) / len2[mask], 0, 1)
                proj_x = x1 + t*dx; proj_y = y1 + t*dy
                dists = np.sqrt((px-proj_x)**2 + (py-proj_y)**2)
                return np.min(dists) <= half_w

            for gx in np.arange(min_x, max_x, res):
                for gy in np.arange(min_y, max_y, res):
                    if in_target(gx, gy):
                        if is_covered(gx, gy):
                            covered_x.append(gx); covered_y.append(gy)
                        else:
                            uncovered_x.append(gx); uncovered_y.append(gy)

        if covered_x:
            ax_cov.scatter(covered_x, covered_y, c='green', s=1, alpha=0.5, marker='s')
        if uncovered_x:
            ax_cov.scatter(uncovered_x, uncovered_y, c='red', s=2, alpha=0.7, marker='s')

        ax_cov.set_aspect('equal')
        ax_cov.set_xlabel('X (m)'); ax_cov.set_ylabel('Y (m)')

    fig.suptitle(f'{name} — Our Custom Algorithm vs Snake Mode', fontsize=14, fontweight='bold')
    plt.tight_layout()
    png_path = f'{rd}/{name}_comparison.png'
    plt.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Rendered: {png_path}')

# 打印汇总表
print("\n" + "="*70)
print("  对比汇总")
print("="*70)
print(f"{'场景':<6} {'模式':<12} {'覆盖率':>8} {'得分':>6} {'转弯':>6} {'重叠率':>8}")
print("-"*55)
for name in names:
    for mode in modes:
        data = load_data(name, mode)
        if data is None:
            continue
        ev = data.get('eval', {})
        cov = ev.get('coverage_rate', 0) or 0
        score = ev.get('single_score', 0) or 0
        turns = ev.get('turn_count', 0) or 0
        overlap = ev.get('overlap_rate', 0) or 0
        label = mode_labels[mode] if mode == 'custom' else 'Snake'
        print(f"{name:<6} {label:<12} {cov:>7.2f}% {score:>5.1f} {turns:>5} {overlap:>7.1f}%")
    print("-"*55)

print(f"\n报告目录: {rd}")
PYRENDER

echo ""
echo "Done! Results: $RESULT_DIR"
ls -la "$RESULT_DIR"/*.png 2>/dev/null || echo "  (no PNG files)"
