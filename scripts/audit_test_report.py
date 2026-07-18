#!/usr/bin/env python3
"""对 F2C 批测 JSON 做不依赖 ROS 的一致性检查。"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys


@dataclass(frozen=True)
class Finding:
    code: str
    severity: str
    message: str


def _path_points(path):
    points = []
    for point in path:
        xy = (float(point["x"]), float(point["y"]))
        if not points or math.dist(points[-1], xy) > 1e-9:
            points.append(xy)
    return points


def _geometry_turn_count(path, angle_threshold_deg=30.0, merge_distance=0.75):
    points = _path_points(path)
    if len(points) < 3:
        return 0

    threshold = math.radians(angle_threshold_deg)
    arc_distance = 0.0
    last_turn_arc = -math.inf
    turns = 0
    for index in range(1, len(points) - 1):
        previous, current, following = points[index - 1:index + 2]
        arc_distance += math.dist(previous, current)
        incoming = math.atan2(current[1] - previous[1], current[0] - previous[0])
        outgoing = math.atan2(following[1] - current[1], following[0] - current[0])
        delta = abs((outgoing - incoming + math.pi) % (2.0 * math.pi) - math.pi)
        if delta > threshold:
            if arc_distance - last_turn_arc > max(0.0, merge_distance):
                turns += 1
            last_turn_arc = arc_distance
    return turns


def _point_segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.dist(point, start)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    closest = (ax + ratio * dx, ay + ratio * dy)
    return math.dist(point, closest)


def _ring_edges(ring):
    points = [tuple(map(float, point[:2])) for point in ring]
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return list(zip(points, points[1:] + points[:1]))


def _point_in_ring(point, ring):
    edges = _ring_edges(ring)
    if not edges:
        return False
    if any(_point_segment_distance(point, start, end) <= 1e-9 for start, end in edges):
        return True

    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in edges:
        if (y1 > y) != (y2 > y):
            crossing_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < crossing_x:
                inside = not inside
    return inside


def _point_in_work_area(point, polygon):
    if not _point_in_ring(point, polygon.get("polygon", [])):
        return False
    return not any(_point_in_ring(point, hole) for hole in polygon.get("holes", []))


def _boundary_clearance(point, polygon):
    rings = [polygon.get("polygon", []), *polygon.get("holes", [])]
    distances = [
        _point_segment_distance(point, start, end)
        for ring in rings
        for start, end in _ring_edges(ring)
    ]
    return min(distances, default=math.inf)


def _sampled_length(points, predicate, sample_step=0.05):
    total = 0.0
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        if length <= 1e-12:
            continue
        sample_count = max(1, math.ceil(length / sample_step))
        sample_length = length / sample_count
        for sample_index in range(sample_count):
            ratio = (sample_index + 0.5) / sample_count
            midpoint = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            if predicate(midpoint):
                total += sample_length
    return total


def _estimated_retrace_length(
        points, sample_step=0.05, spatial_resolution=0.025,
        angle_resolution_deg=10.0):
    seen = set()
    duplicate_length = 0.0
    angle_resolution = math.radians(angle_resolution_deg)
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        if length <= 1e-12:
            continue
        sample_count = max(1, math.ceil(length / sample_step))
        sample_length = length / sample_count
        direction = math.atan2(end[1] - start[1], end[0] - start[0]) % math.pi
        for sample_index in range(sample_count):
            ratio = (sample_index + 0.5) / sample_count
            midpoint = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            key = (
                round(midpoint[0] / spatial_resolution),
                round(midpoint[1] / spatial_resolution),
                round(direction / angle_resolution),
            )
            if key in seen:
                duplicate_length += sample_length
            else:
                seen.add(key)
    return duplicate_length


def audit_report(report, polygon, robot_width=0.75, merge_distance=0.75):
    """检查单场景报告，返回按严重程度分类的发现。"""
    findings = []
    path_points = _path_points(report.get("path", []))
    if len(path_points) < 2:
        findings.append(Finding(
            code="PATH_MISSING",
            severity="error",
            message="报告没有可审计的最终路径。",
        ))
    if not report.get("swaths"):
        findings.append(Finding(
            code="EMPTY_SWATHS",
            severity="error",
            message="报告没有 swath 证据，无法核对覆盖线与路径。",
        ))
    reported_turns = report.get("eval", {}).get("turn_count")
    if reported_turns is not None:
        geometry_turns = _geometry_turn_count(
            report.get("path", []), merge_distance=merge_distance)
        if int(reported_turns) != geometry_turns:
            findings.append(Finding(
                code="TURN_COUNT_MISMATCH",
                severity="error",
                message=(
                    f"报告转弯数为 {reported_turns}，最终路径几何复算为 {geometry_turns}。"
                ),
            ))
    outside_length = _sampled_length(
        path_points, lambda point: not _point_in_work_area(point, polygon))
    if outside_length > 0.05:
        findings.append(Finding(
            code="CENTERLINE_OUTSIDE",
            severity="error" if outside_length >= 0.5 else "warning",
            message=f"路径中心线约有 {outside_length:.2f} m 位于作业区域之外。",
        ))
    required_clearance = max(0.0, float(robot_width)) * 0.5
    unsafe_length = _sampled_length(
        path_points,
        lambda point: (
            not _point_in_work_area(point, polygon)
            or _boundary_clearance(point, polygon) + 1e-9 < required_clearance
        ),
    )
    if required_clearance > 0.0 and unsafe_length > 0.05:
        findings.append(Finding(
            code="FOOTPRINT_CLEARANCE",
            severity="error",
            message=(
                f"按机器人宽度 {robot_width:.2f} m 估算，约 {unsafe_length:.2f} m "
                "路径没有足够边界净空。"
            ),
        ))
    segment_lengths = [
        math.dist(start, end) for start, end in zip(path_points, path_points[1:])
    ]
    tiny_count = sum(length < 0.01 for length in segment_lengths)
    tiny_ratio = tiny_count / len(segment_lengths) if segment_lengths else 0.0
    if tiny_ratio >= 0.25:
        findings.append(Finding(
            code="TINY_SEGMENTS",
            severity="warning",
            message=(
                f"{tiny_count}/{len(segment_lengths)} 个路径段短于 1 cm "
                f"（{tiny_ratio:.1%}），可能导致控制器抖动。"
            ),
        ))
    total_length = sum(segment_lengths)
    retrace_length = _estimated_retrace_length(path_points)
    if retrace_length > max(0.5, total_length * 0.005):
        findings.append(Finding(
            code="RETRACE_LENGTH",
            severity="warning",
            message=(
                f"估算有 {retrace_length:.2f} m 路径沿同一位置和方向重复经过，"
                "可能存在边界往返或冗余连接。"
            ),
        ))
    connections = report.get("connections", [])
    if connections and any(not connection.get("source") for connection in connections):
        findings.append(Finding(
            code="CONNECTION_PROVENANCE",
            severity="warning",
            message="连接线没有来源标记，无法判断它是真实路径还是由端点推测的直线。",
        ))
    return findings


_POINT_PATTERN = re.compile(
    r"\[\s*(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,\s*"
    r"(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*\]"
)


def load_polygon_yaml(path):
    """读取项目场景 YAML 中 polygon/holes 使用的简单坐标列表。"""
    polygon = []
    holes = []
    section = None
    current_hole = None
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        stripped = line.strip()
        if stripped.startswith("polygon:"):
            section = "polygon"
            continue
        if stripped.startswith("holes:"):
            section = "holes"
            current_hole = None
            continue
        if not stripped or section not in {"polygon", "holes"}:
            continue
        match = _POINT_PATTERN.search(stripped)
        if not match:
            if not line.startswith(" "):
                section = None
            continue
        point = [float(match.group(1)), float(match.group(2))]
        if section == "polygon":
            polygon.append(point)
        elif stripped.startswith("- -"):
            current_hole = [point]
            holes.append(current_hole)
        elif current_hole is not None:
            current_hole.append(point)
    return {"polygon": polygon, "holes": holes}


def _find_polygon_file(repo_root, scenario):
    search_dirs = [
        repo_root / "src" / "yingshi_robot" / "test_polygons",
        repo_root / "src" / "yingshi_robot" / "config" / "f2c_areas",
    ]
    patterns = [f"{scenario}.yaml", f"{scenario}_*.yaml", f"{scenario}*.yaml"]
    for directory in search_dirs:
        for pattern in patterns:
            matches = sorted(directory.glob(pattern))
            if matches:
                return matches[0]
    return None


def audit_batch(batch_dir, repo_root, robot_width=0.75, merge_distance=0.75):
    results = []
    report_paths = list(Path(batch_dir).glob("*_data.json"))
    report_paths.sort(key=lambda path: (
        0 if re.fullmatch(r"S\d+", path.stem.removesuffix("_data")) else 1,
        int(path.stem.removesuffix("_data")[1:])
        if re.fullmatch(r"S\d+", path.stem.removesuffix("_data")) else 0,
        path.stem.lower(),
    ))
    for report_path in report_paths:
        scenario = report_path.stem.removesuffix("_data")
        polygon_path = _find_polygon_file(Path(repo_root), scenario)
        if polygon_path is None:
            findings = [Finding(
                code="POLYGON_FILE_MISSING",
                severity="error",
                message=f"找不到场景 {scenario} 对应的 polygon YAML。",
            )]
        else:
            try:
                report = json.loads(report_path.read_text(encoding="utf-8"))
                polygon = load_polygon_yaml(polygon_path)
                findings = audit_report(
                    report,
                    polygon,
                    robot_width=robot_width,
                    merge_distance=merge_distance,
                )
            except (OSError, ValueError, json.JSONDecodeError) as error:
                findings = [Finding(
                    code="REPORT_READ_ERROR",
                    severity="error",
                    message=f"无法读取报告或场景文件：{error}",
                )]
        results.append((scenario, findings))
    return results


def format_markdown(batch_dir, results):
    error_count = sum(
        finding.severity == "error" for _, findings in results for finding in findings)
    warning_count = sum(
        finding.severity == "warning" for _, findings in results for finding in findings)
    lines = [
        "# F2C 测试报告体检",
        "",
        f"- 批次：`{Path(batch_dir).name}`",
        f"- 场景：{len(results)}",
        f"- 严重问题：{error_count}",
        f"- 警告：{warning_count}",
        "",
        "> 几何长度按 5 cm 间隔采样；机器人净空按宽度的一半估算。",
        "> 重走长度是栅格化下界，用于定位明显往返，不等同于精确扫掠面积重叠率。",
        "",
    ]
    for scenario, findings in results:
        lines.append(f"## {scenario}")
        lines.append("")
        if not findings:
            lines.append("- PASS：未发现结构或几何异常。")
        for finding in findings:
            label = "ERROR" if finding.severity == "error" else "WARN"
            lines.append(f"- **{label} `{finding.code}`**：{finding.message}")
        lines.append("")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description="审计 F2C 批量测试 JSON 报告")
    parser.add_argument("batch_dir", help="包含 *_data.json 的批次目录")
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--robot-width", type=float, default=0.75)
    parser.add_argument(
        "--merge-distance",
        type=float,
        default=0.75,
        help="相邻方向变化归并为同一转弯的最小路径距离（米）",
    )
    parser.add_argument("--markdown", help="可选的 Markdown 输出路径")
    args = parser.parse_args(argv)

    results = audit_batch(
        args.batch_dir,
        args.repo_root,
        robot_width=args.robot_width,
        merge_distance=args.merge_distance,
    )
    markdown = format_markdown(args.batch_dir, results)
    print(markdown)
    if args.markdown:
        output_path = Path(args.markdown)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(markdown + "\n", encoding="utf-8")
    return 1 if any(
        finding.severity == "error" for _, findings in results for finding in findings
    ) else 0


if __name__ == "__main__":
    sys.exit(main())
