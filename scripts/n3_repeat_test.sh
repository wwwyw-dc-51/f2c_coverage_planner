#!/usr/bin/env bash
# N3_dense 重复规划测试：判断路线差异是否来自启发式路由波动。
#
# 用法：
#   bash scripts/n3_repeat_test.sh
#   bash scripts/n3_repeat_test.sh --runs 3 --render
#   bash scripts/n3_repeat_test.sh --runs 5 --output test_results/n3_repeat_manual
#
# 输出：每轮的 planner 日志、可视化 JSON、汇总 summary.json/summary.txt。

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YAML="$WS/src/yingshi_robot/test_polygons/dense_shelves.yaml"
RUNS=5
TIMEOUT=180
OUT=""
RENDER=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --runs) RUNS="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --output) OUT="$2"; shift 2 ;;
        --render) RENDER=1; shift ;;
        -h|--help)
            sed -n '1,12p' "$0"
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

[[ "$RUNS" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: --runs must be positive" >&2; exit 2; }
[[ "$TIMEOUT" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: --timeout must be positive" >&2; exit 2; }
[[ -f "$YAML" ]] || { echo "ERROR: missing N3 YAML: $YAML" >&2; exit 1; }

if [[ -z "$OUT" ]]; then
    OUT="$WS/test_results/n3_repeat_$(date +%m%d_%H%M%S)"
elif [[ "$OUT" != /* ]]; then
    OUT="$WS/$OUT"
fi
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$WS/install/fields2cover/lib:$WS/install/lib:$WS/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"

PARAMS=(
    -p use_planner_core:=true
    -p traversability_enabled:=true
    -p robot_width:=0.75
    -p coverage_width:=0.90
    -p mid_hl_width_ratio:=0.20
    -p no_hl_width_ratio:=0.0
    -p min_hole_area:=0.0
    -p cspace_clearance_margin:=0.0
    -p max_excluded_area_ratio:=0.05
    -p decomposition_angle:=0.0
    -p swath_endpoint_shrink_distance:=0.03
    -p min_swath_length:=0.5
    -p path_resolution:=0.1
    -p boundary_type:=closed
    -p use_optimized_planner:=true
    -p swath_angle_optimization:=true
    -p decomposition_enabled:=true
    -p filter_tiny_cells:=true
    -p use_sweep_decomp:=true
    -p merge_angle_threshold:=60.0
    -p swath_order_type:=boustrophedon
    -p path_simplify_enabled:=true
    -p path_simplify_tolerance:=0.05
    -p path_simplify_turn_threshold:=0.15
    -p turn_planner_type:=direct
    -p swath_overlap_ratio:=0.03
    -p eval_enable_report:=true
    -p eval_use_grid_method:=true
    -p eval_grid_resolution:=0.1
    -p eval_coverage_threshold:=0.99
)

VIS="/tmp/f2c_vis_polygon_1.json"
GRID="/tmp/f2c_grid_polygon_1.json"
PID=""

stop_planner() {
    if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
        kill "$PID" 2>/dev/null || true
        wait "$PID" 2>/dev/null || true
    fi
    PID=""
}
trap stop_planner EXIT INT TERM

run_once() {
    local index="$1"
    local tag="run_$(printf '%02d' "$index")"
    local log="$OUT/${tag}_planner.log"
    local data="$OUT/${tag}_data.json"

    rm -f "$VIS" "$GRID" "$data"
    echo "=== N3 $tag/$RUNS $(date '+%Y-%m-%d %H:%M:%S') ==="

    ros2 run yingshi_robot polygon_planner_node --ros-args "${PARAMS[@]}" >"$log" 2>&1 &
    PID=$!
    sleep 3
    if ! kill -0 "$PID" 2>/dev/null; then
        echo "FAIL: planner exited early; see $log" >&2
        return 1
    fi

    set +e
    python3 - "$YAML" "$log" "$TIMEOUT" <<'PY_CAPTURE'
import os, sys, time, yaml, rclpy
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

yaml_path, log_path, timeout_text = sys.argv[1:]
timeout = int(timeout_text)
rclpy.init()
node = rclpy.create_node('n3_repeat_capture')
received = False

def on_path(message):
    global received
    if message.poses:
        received = True

node.create_subscription(Path, '/planned2_path_1', on_path, 10)
poly_pub = node.create_publisher(Polygon, '/input_polygon_1', 10)
holes_pub = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

with open(yaml_path, encoding='utf-8') as handle:
    area = yaml.safe_load(handle)

poly = Polygon()
for point in area['polygon']:
    poly.points.append(Point32(x=float(point[0]), y=float(point[1]), z=0.0))
holes = Polygon()
for index, hole in enumerate(area.get('holes', [])):
    if index:
        holes.points.append(Point32(x=1e10, y=0.0, z=0.0))
    for point in hole:
        holes.points.append(Point32(x=float(point[0]), y=float(point[1]), z=0.0))

time.sleep(1.0)
for _ in range(5):
    holes_pub.publish(holes)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.3)
    poly_pub.publish(poly)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.5)
        if received:
            break
    if received:
        break

deadline = time.monotonic() + timeout
complete = False
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if os.path.exists(log_path):
        text = open(log_path, encoding='utf-8', errors='replace').read()
        if 'PLANNERCORE_COMPLETE polygon=1' in text:
            complete = True
            break
    time.sleep(0.3)

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if received and complete else 1)
PY_CAPTURE
    local status=$?
    set -e

    sleep 2
    [[ -f "$VIS" ]] && cp "$VIS" "$OUT/${tag}_vis.json"
    [[ -f "$GRID" ]] && cp "$GRID" "$OUT/${tag}_grid.json"

    python3 - "$log" "$OUT/${tag}_vis.json" "$OUT/${tag}_grid.json" "$data" "$status" <<'PY_DATA'
import hashlib, json, math, re, sys
from pathlib import Path

log_path, vis_path, grid_path, output_path, status = sys.argv[1:]
text = Path(log_path).read_text(encoding='utf-8', errors='replace')
try:
    vis = json.loads(Path(vis_path).read_text(encoding='utf-8'))
except (OSError, json.JSONDecodeError):
    vis = {}

def extract(pattern, cast=float, flags=0):
    values = re.findall(pattern, text, flags=flags)
    return cast(values[-1]) if values else None

evaluation = {
    'coverage_rate': extract(r'(?<!原始)(?<!有效)覆盖率[:：\s]*([\d.]+)%'),
    'effective_coverage_rate': extract(r'有效覆盖率[:：\s]*([\d.]+)%'),
    'single_score': extract(r'综合得分[:：\s]*([\d.]+)'),
    'total_distance': extract(r'路径总长[:：\s]*([\d.]+)'),
    'overlap_rate': extract(r'重叠率[:：\s]*([\d.]+)%'),
    'turn_count': extract(r'^转弯次数[:：\s]*(\d+)\s*$', int, re.MULTILINE),
}

def point_key(point):
    if isinstance(point, dict):
        return (round(float(point.get('x', 0)), 9), round(float(point.get('y', 0)), 9))
    return (round(float(point[0]), 9), round(float(point[1]), 9))

def swath_key(swath):
    return tuple(point_key(point) for point in swath.get('points', []))

def connection_key(connection):
    return tuple(point_key(point) for point in connection.get('path', []))

def digest(value):
    raw = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
    return hashlib.sha1(raw).hexdigest()[:12]

swaths = [swath_key(item) for item in vis.get('swaths', [])]
connections = [connection_key(item) for item in vis.get('connections', [])]
payload = {
    'scenario': 'N3_dense',
    'run_complete': status == '0',
    'cells': vis.get('cells', []),
    'swaths': vis.get('swaths', []),
    'connections': vis.get('connections', []),
    'path': vis.get('path', []),
    'eval': evaluation,
    'signatures': {
        'ordered_swaths': digest(swaths),
        'swath_set': digest(sorted(swaths)),
        'connections': digest(connections),
    },
    'batch_status': {
        'complete': status == '0',
        'visualization_artifact_created': bool(vis),
        'grid_artifact_created': Path(grid_path).exists(),
    },
}
Path(output_path).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding='utf-8')
print(json.dumps({
    'complete': payload['run_complete'],
    'cells': len(payload['cells']),
    'swaths': len(payload['swaths']),
    'connections': len(payload['connections']),
    **evaluation,
}, ensure_ascii=False))
PY_DATA

    stop_planner
    if [[ "$status" -ne 0 ]]; then
        echo "FAIL: $tag did not complete; see $log" >&2
        return 1
    fi

    if [[ "$RENDER" -eq 1 ]]; then
        python3 "$WS/scripts/render_coverage.py" N3_dense "$YAML" "$data" \
            "$OUT/$tag"_coverage.png "$OUT/$tag"_grid.json || true
        python3 "$WS/scripts/render_coverage.py" N3_dense "$YAML" "$data" \
            "$OUT/$tag"_cells.png --mode cells || true
    fi
}

echo "N3 repeat test: runs=$RUNS timeout=$TIMEOUT output=$OUT"
failures=0
for ((index=1; index<=$RUNS; index++)); do
    if ! run_once "$index"; then
        failures=$((failures + 1))
    fi
done

python3 - "$OUT" <<'PY_SUMMARY'
import json, math, statistics, sys
from pathlib import Path

root = Path(sys.argv[1])
records = []
for path in sorted(root.glob('run_*_data.json')):
    data = json.loads(path.read_text(encoding='utf-8'))
    ev = data.get('eval', {})
    records.append({
        'run': path.stem.replace('_data', ''),
        'complete': bool(data.get('run_complete')),
        'cells': len(data.get('cells', [])),
        'swaths': len(data.get('swaths', [])),
        'connections_count': len(data.get('connections', [])),
        'distance': ev.get('total_distance'),
        'turns': ev.get('turn_count'),
        'overlap': ev.get('overlap_rate'),
        'score': ev.get('single_score'),
        **data.get('signatures', {}),
    })

complete = [item for item in records if item['complete']]
def metric(name):
    return [x[name] for x in complete
            if isinstance(x[name], (int, float)) and math.isfinite(x[name])]

summary = {
    'scenario': 'N3_dense',
    'requested_runs': len(records),
    'completed_runs': len(complete),
    'records': records,
    'same_cells': len({x['cells'] for x in complete}) <= 1 if complete else False,
    'same_swath_count': len({x['swaths'] for x in complete}) <= 1 if complete else False,
    'same_swath_set': len({x['swath_set'] for x in complete}) <= 1 if complete else False,
    'route_order_changed': len({x['ordered_swaths'] for x in complete}) > 1 if complete else False,
    'connections_changed': len({x['connections'] for x in complete}) > 1 if complete else False,
}
for name in ('distance', 'turns', 'overlap', 'score'):
    values = metric(name)
    summary[name] = {
        'min': min(values) if values else None,
        'median': statistics.median(values) if values else None,
        'max': max(values) if values else None,
        'range': max(values) - min(values) if values else None,
        'stdev': statistics.pstdev(values) if len(values) > 1 else 0.0,
    }

if len(complete) != len(records):
    summary['classification'] = 'incomplete'
elif summary['same_swath_set'] and summary['connections_changed']:
    summary['classification'] = 'route_variation_same_swaths'
else:
    summary['classification'] = 'stable_route'

(root / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
lines = [
    'N3 repeat test summary',
    f"completed: {summary['completed_runs']}/{summary['requested_runs']}",
    f"classification: {summary['classification']}",
    f"same cells={summary['same_cells']} same swath set={summary['same_swath_set']} "
    f"route order changed={summary['route_order_changed']} "
    f"connections changed={summary['connections_changed']}",
    '',
    'run       cells swaths conns distance(m) turns overlap(%) score',
]
for x in records:
    lines.append(
        f"{x['run']:8s} {x['cells']:5d} {x['swaths']:6d} {x['connections_count']:5d} "
        f"{x['distance'] if x['distance'] is not None else float('nan'):11.3f} "
        f"{x['turns'] if x['turns'] is not None else float('nan'):5.0f} "
        f"{x['overlap'] if x['overlap'] is not None else float('nan'):10.3f} "
        f"{x['score'] if x['score'] is not None else float('nan'):5.1f}")
for name in ('distance', 'turns', 'overlap', 'score'):
    s = summary[name]
    lines.append(f"{name:8s} min={s['min']} median={s['median']} max={s['max']} range={s['range']} stdev={s['stdev']}")
(root / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
print('\n'.join(lines))
print(f"\nsummary: {root / 'summary.txt'}")
PY_SUMMARY

if [[ "$failures" -gt 0 ]]; then
    echo "WARNING: $failures run(s) failed; inspect *_planner.log" >&2
    exit 1
fi
echo "Done: $OUT"
