#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RVIZ_CONFIG="${WORKSPACE_DIR}/src/yingshi_robot/config/f2c_planner.rviz"
DEFAULT_AREA_FILE="${WORKSPACE_DIR}/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"
AREA_FILE="${1:-${DEFAULT_AREA_FILE}}"
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
  echo "  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TUTORIALS=OFF -DBUILD_TESTS=OFF -DBUILD_PYTHON=OFF" >&2
  exit 1
fi

export ROS_LOG_DIR
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/src/Fields2Cover/third_party/ortools-src/lib:${LD_LIBRARY_PATH:-}"
cd "${WORKSPACE_DIR}"

# 从 YAML 自动提取 boundary_type
BOUNDARY_TYPE=$(python3 -c "
import yaml, sys
with open('${AREA_FILE}', 'r') as f:
    data = yaml.safe_load(f)
print(data.get('boundary_type', 'closed'))
" 2>/dev/null || echo "closed")
echo "Boundary type from YAML: ${BOUNDARY_TYPE}"

ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p robot_width:=0.95 \
  -p min_turning_radius:=0.1 \
  -p max_diff_curv:=0.3 \
  -p coverage_width:=0.45 \
  -p mid_hl_width_ratio:=0.25 \
  -p no_hl_width_ratio:=0.0 \
  -p min_hole_area:=1.0 \
  -p decomposition_angle:=0.0 \
  -p swath_endpoint_shrink_distance:=0.25 \
  -p min_swath_length:=0.5 \
  -p boundary_type:="${BOUNDARY_TYPE}" &
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

polygon_msg = make_polygon(polygon)
holes_msg = make_holes(holes)

time.sleep(0.8)
for _ in range(3):
    holes_pub.publish(holes_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)

for _ in range(3):
    polygon_pub.publish(polygon_msg)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.2)

print(
    f"Published area from {area_file}: "
    f"{len(polygon_msg.points)} polygon points, {len(holes)} holes"
)

node.destroy_node()
rclpy.shutdown()
PY

echo
echo "F2C RViz demo is running."
echo "Parameters match polygon_planner.launch.py."
echo "Area file: ${AREA_FILE}"
echo "Press Ctrl-C in this terminal to stop polygon_planner_node and RViz."

wait "${PLANNER_PID}"
