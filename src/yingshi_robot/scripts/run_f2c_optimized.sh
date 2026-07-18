#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 智能定位 workspace：install 路径下用前缀截取，源文件路径下往上3级
if [[ "$SCRIPT_DIR" == */install/* ]]; then
    WORKSPACE_DIR="${SCRIPT_DIR%%/install/*}"
else
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi
RVIZ_CONFIG="${WORKSPACE_DIR}/src/yingshi_robot/config/f2c_planner.rviz"
DEFAULT_AREA_FILE="${WORKSPACE_DIR}/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"

# 场景名 → YAML 路径映射
TEST_DIR="${WORKSPACE_DIR}/src/yingshi_robot/test_polygons"
F2C_DIR="${WORKSPACE_DIR}/src/yingshi_robot/config/f2c_areas"
declare -A SCENE_MAP=(
    ["S1"]="${TEST_DIR}/S1_S1_convex_rect.yaml"
    ["S2"]="${TEST_DIR}/S2_S2_L_shaped.yaml"
    ["S3"]="${TEST_DIR}/S3_S3_with_holes.yaml"
    ["S4"]="${TEST_DIR}/S4_S4_narrow_corridor.yaml"
    ["S5"]="${TEST_DIR}/S5_S5_irregular.yaml"
    ["S6"]="${TEST_DIR}/S6_S6_multi_region.yaml"
    ["notched"]="${F2C_DIR}/notched_10m_with_center_hole.yaml"
)

if [[ "${1:-}" == "--help" ]] || [[ "${1:-}" == "-h" ]]; then
    echo "用法: $0 [场景名|YAML路径]"
    echo ""
    echo "可用场景名:"
    echo "  S1      — 矩形 20×15m"
    echo "  S2      — L 形"
    echo "  S3      — 含孔洞"
    echo "  S4      — 窄走廊"
    echo "  S5      — 不规则多边形"
    echo "  S6      — 多区域"
    echo "  notched — 缺口孔洞 (默认)"
    echo ""
    echo "示例:"
    echo "  $0          # 默认 notched"
    echo "  $0 S5       # 查看 S5 不规则多边形"
    echo "  $0 /path/to/custom.yaml  # 自定义场景"
    exit 0
fi

AREA_FILE="${1:-}"
if [[ -z "${AREA_FILE}" ]]; then
    AREA_FILE="${DEFAULT_AREA_FILE}"
elif [[ -n "${SCENE_MAP[${AREA_FILE}]:-}" ]]; then
    AREA_FILE="${SCENE_MAP[${AREA_FILE}]}"
fi
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"

PLANNER_PID=""
RVIZ_PID=""

source_env_file() {
  local env_file="$1"
  local restore_nounset=0

  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi

  source "${env_file}"

  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

cleanup() {
  trap - INT TERM EXIT

  if [[ -n "${RVIZ_PID}" ]] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi

  if [[ -n "${PLANNER_PID}" ]] && kill -0 "${PLANNER_PID}" 2>/dev/null; then
    kill "${PLANNER_PID}" 2>/dev/null || true
  fi

  wait "${RVIZ_PID}" 2>/dev/null || true
  wait "${PLANNER_PID}" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "Missing RViz config: ${RVIZ_CONFIG}" >&2
  exit 1
fi

if [[ ! -f "${AREA_FILE}" ]]; then
  echo "Missing area file: ${AREA_FILE}" >&2
  exit 1
fi

mkdir -p "${ROS_LOG_DIR}"

source_env_file /opt/ros/humble/setup.bash

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source_env_file "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "Missing ${WORKSPACE_DIR}/install/setup.bash" >&2
  echo "Build and source the workspace first:" >&2
  echo "  cd ${WORKSPACE_DIR}" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" >&2
  exit 1
fi

export ROS_LOG_DIR
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/src/Fields2Cover/build/_deps/steering_functions-build:${WORKSPACE_DIR}/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"
cd "${WORKSPACE_DIR}"

# 从 YAML 自动提取 boundary_type
BOUNDARY_TYPE=$(python3 -c "
import yaml, sys
with open('${AREA_FILE}', 'r') as f:
    data = yaml.safe_load(f)
print(data.get('boundary_type', 'closed'))
" 2>/dev/null || echo "closed")
echo "Boundary type from YAML: ${BOUNDARY_TYPE}"

# 诊断日志导出
DIAG_DIR="${WORKSPACE_DIR}/src/yingshi_robot/diag_logs"
mkdir -p "${DIAG_DIR}"
DIAG_FILE="${DIAG_DIR}/diag_$(date +%m%d_%H%M%S).log"
echo "Diagnostic log: ${DIAG_FILE}"

ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p use_planner_core:=true \
  -p robot_width:=0.75 \
  -p coverage_width:=0.90 \
  -p mid_hl_width_ratio:=0.20 \
  -p no_hl_width_ratio:=0.0 \
  -p min_hole_area:=0.0 \
  -p traversability_enabled:=true \
  -p cspace_clearance_margin:=0.0 \
  -p max_excluded_area_ratio:=0.05 \
  -p decomposition_angle:=0.0 \
  -p swath_endpoint_shrink_distance:=0.03 \
  -p min_swath_length:=0.5 \
  -p max_diff_curv:=0.3 \
  -p path_resolution:=0.1 \
  -p boundary_type:="${BOUNDARY_TYPE}" \
  -p use_optimized_planner:=true \
  -p swath_angle_optimization:=true \
  -p decomposition_enabled:=true \
  -p filter_tiny_cells:=true \
  -p path_simplify_enabled:=true \
  -p path_simplify_tolerance:=0.05 \
  -p path_simplify_turn_threshold:=0.15 \
  -p turn_planner_type:=direct \
  -p swath_overlap_ratio:=0.03 \
  -p swath_order_type:=boustrophedon \
  -p use_sweep_decomp:=true \
  -p merge_angle_threshold:=60.0 \
  -p eval_enable_report:=true \
  -p eval_use_grid_method:=true \
  -p eval_grid_resolution:=0.1 \
  -p eval_coverage_threshold:=0.99 > "${DIAG_FILE}" 2>&1 &

PLANNER_PID=$!

sleep 1

rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!

sleep 2

python3 - "${AREA_FILE}" <<'PY'
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path


HOLE_SEPARATOR = 1e10


def make_point(point):
    if len(point) < 2:
        raise ValueError(f"Point must contain at least x and y: {point!r}")
    msg = Point32()
    msg.x = float(point[0])
    msg.y = float(point[1])
    msg.z = float(point[2]) if len(point) > 2 else 0.0
    return msg


def make_polygon(points):
    msg = Polygon()
    msg.points = [make_point(point) for point in points]
    return msg


def make_holes(holes):
    msg = Polygon()
    for hole_index, hole in enumerate(holes):
        if hole_index > 0:
            msg.points.append(Point32(x=HOLE_SEPARATOR, y=0.0, z=0.0))
        msg.points.extend(make_point(point) for point in hole)
    return msg


area_file = sys.argv[1]
with open(area_file, "r", encoding="utf-8") as f:
    area = yaml.safe_load(f)

polygon = area.get("polygon", [])
holes = area.get("holes", [])

if len(polygon) < 3:
    raise ValueError("Area file must contain polygon with at least 3 points")

for idx, hole in enumerate(holes):
    if len(hole) < 3:
        raise ValueError(f"holes[{idx}] must contain at least 3 points")

rclpy.init()
node = rclpy.create_node("f2c_area_yaml_publisher")
polygon_pub = node.create_publisher(Polygon, "/input_polygon_1", 10)
holes_pub = node.create_publisher(Polygon, "/input_polygon_1_holes", 10)

# 订阅规划结果，用于确认信号已被处理
plan_received = False
def on_plan(msg):
    global plan_received
    if len(msg.poses) > 0:
        plan_received = True
plan_sub = node.create_subscription(Path, "/planned2_path_1", on_plan, 10)
component_plan_subs = [
    node.create_subscription(
        Path, f"/planned2_path_1_component_{index}", on_plan, 10)
    for index in range(1, 65)
]

polygon_msg = make_polygon(polygon)
holes_msg = make_holes(holes)

time.sleep(1.0)

# 带确认的重发机制：最多发 5 次，检测到规划结果立即停止
for attempt in range(5):
    holes_pub.publish(holes_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)
    polygon_pub.publish(polygon_msg)

    # 等待规划完成，最长等 3 秒
    waited = 0.0
    while waited < 3.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        waited += 0.2
        if plan_received:
            break

    if plan_received:
        print(f"Plan confirmed after attempt {attempt + 1}")
        break
    else:
        print(f"Attempt {attempt + 1}: no plan received, retrying...")
else:
    print("Warning: no plan received after 5 attempts")

print(
    f"Published area from {area_file}: "
    f"{len(polygon_msg.points)} polygon points, {len(holes)} holes"
)

node.destroy_node()
rclpy.shutdown()
PY

echo
echo "F2C Optimized RViz demo is running."
echo "turn_planner=direct | overlap=3% | all optimizations ON"
echo "Area file: ${AREA_FILE}"
echo "Diagnostic log: ${DIAG_FILE}"
echo "Press Ctrl-C to stop."

wait "${PLANNER_PID}"
