#!/usr/bin/env python3
"""校验单场景批测产物是否满足发布门控。"""

import argparse
import json
import math
from pathlib import Path
import sys


REQUIRED_BATCH_STATUS = (
    "plan_received",
    "evaluation_completed",
    "visualization_artifact_created",
    "grid_artifact_created",
)
REQUIRED_EVAL_METRICS = (
    "coverage_rate",
    "raw_coverage_rate",
    "effective_coverage_rate",
    "single_score",
    "uncovered_area",
    "unreachable_area",
    "reachable_uncovered_area",
    "total_distance",
    "work_ratio",
    "turn_count",
    "overlap_rate",
    "planning_time_ms",
    "net_area",
)
BOUNDED_EVAL_METRICS = {
    "coverage_rate": (0.0, 100.0),
    "raw_coverage_rate": (0.0, 100.0),
    "effective_coverage_rate": (0.0, 100.0),
    "single_score": (0.0, 100.0),
}
NON_NEGATIVE_EVAL_METRICS = (
    "uncovered_area",
    "unreachable_area",
    "reachable_uncovered_area",
    "total_distance",
    "work_ratio",
    "overlap_rate",
    "planning_time_ms",
)
CSPACE_AREA_FIELDS = (
    "original_area",
    "reachable_area",
    "excluded_area",
)
CSPACE_RATIO_FIELDS = (
    "excluded_ratio",
    "max_ratio",
)


def _is_finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _validate_evaluation(evaluation, prefix, coverage_threshold, errors):
    if not isinstance(evaluation, dict):
        errors.append(f"{prefix} 缺失或不是对象")
        return
    for metric_name in REQUIRED_EVAL_METRICS:
        if not _is_finite_number(evaluation.get(metric_name)):
            errors.append(
                f"{prefix}.{metric_name} 缺失或不是有限数值")

    for metric_name, (minimum, maximum) in BOUNDED_EVAL_METRICS.items():
        value = evaluation.get(metric_name)
        if _is_finite_number(value) and not minimum <= value <= maximum:
            errors.append(
                f"{prefix}.{metric_name}={value} 超出范围 "
                f"[{minimum}, {maximum}]")
    for metric_name in NON_NEGATIVE_EVAL_METRICS:
        value = evaluation.get(metric_name)
        if _is_finite_number(value) and value < 0.0:
            errors.append(f"{prefix}.{metric_name}={value} 不能为负数")

    turn_count = evaluation.get("turn_count")
    if _is_finite_number(turn_count) and (
            not isinstance(turn_count, int) or turn_count < 0):
        errors.append(f"{prefix}.turn_count 必须是非负整数")
    net_area = evaluation.get("net_area")
    if _is_finite_number(net_area) and net_area <= 0.0:
        errors.append(f"{prefix}.net_area 必须大于零")

    effective_coverage_rate = evaluation.get("effective_coverage_rate")
    if _is_finite_number(effective_coverage_rate) and (
            effective_coverage_rate + 1e-9 < coverage_threshold * 100.0):
        errors.append(
            f"{prefix}.effective_coverage_rate={effective_coverage_rate:.2f}% 低于门槛 "
            f"{coverage_threshold * 100.0:.2f}%")


def validate_report(report, coverage_threshold=0.99):
    """返回报告违反门控条件的原因；空列表表示通过。"""
    if not 0.0 <= coverage_threshold <= 1.0:
        raise ValueError("coverage_threshold 必须位于 [0, 1]")
    if not isinstance(report, dict):
        return ["报告根节点必须是 JSON 对象"]

    errors = []
    batch_status = report.get("batch_status")
    if not isinstance(batch_status, dict):
        batch_status = {}
        errors.append("batch_status 缺失或不是对象")
    for status_name in REQUIRED_BATCH_STATUS:
        if batch_status.get(status_name) is not True:
            errors.append(f"batch_status.{status_name} 未成功")

    cspace_hint = report.get("cspace")
    component_count_hint = (
        cspace_hint.get("component_count")
        if isinstance(cspace_hint, dict) else None)
    is_multi_component = (
        isinstance(component_count_hint, int) and
        not isinstance(component_count_hint, bool) and
        component_count_hint > 1)

    path = report.get("path")
    if is_multi_component:
        if path not in (None, []):
            errors.append("多分量报告的顶层 path 必须为空，禁止跨区展平")
        component_paths = report.get("component_paths")
        if (not isinstance(component_paths, list) or
                len(component_paths) != component_count_hint):
            errors.append(
                "component_paths 数量与 cspace.component_count 不一致")
        else:
            for index, component_path in enumerate(component_paths):
                if not isinstance(component_path, list) or len(component_path) < 2:
                    errors.append(
                        f"component_paths[{index}] 缺失或不足两个点")
    elif not isinstance(path, list) or len(path) < 2:
        errors.append("path 缺失或不足两个点")
    swaths = report.get("swaths")
    if not isinstance(swaths, list) or not swaths:
        errors.append("swaths 缺失或为空")

    if is_multi_component:
        component_evaluations = report.get("component_evals")
        if (not isinstance(component_evaluations, list) or
                len(component_evaluations) != component_count_hint):
            errors.append(
                "component_evals 数量与 cspace.component_count 不一致")
        else:
            for index, evaluation in enumerate(component_evaluations):
                _validate_evaluation(
                    evaluation, f"component_evals[{index}]",
                    coverage_threshold, errors)
    else:
        _validate_evaluation(
            report.get("eval"), "eval", coverage_threshold, errors)

    cspace = report.get("cspace")
    if not isinstance(cspace, dict) or not cspace:
        errors.append("cspace 缺失或为空")
        cspace = {}
    # 生产批测必须启用 traversability，C-space 是结果完整性的必需证据。
    if cspace:
        if cspace.get("valid") is not True:
            errors.append("cspace.valid 未成功")
        for field_name in CSPACE_AREA_FIELDS + CSPACE_RATIO_FIELDS:
            if not _is_finite_number(cspace.get(field_name)):
                errors.append(f"cspace.{field_name} 缺失或不是有限数值")

        for field_name in CSPACE_AREA_FIELDS:
            value = cspace.get(field_name)
            if _is_finite_number(value) and value < 0.0:
                errors.append(f"cspace.{field_name}={value} 不能为负数")
        original_area = cspace.get("original_area")
        reachable_area = cspace.get("reachable_area")
        excluded_area = cspace.get("excluded_area")
        excluded_ratio = cspace.get("excluded_ratio")
        max_ratio = cspace.get("max_ratio")
        if _is_finite_number(original_area) and original_area <= 0.0:
            errors.append("cspace.original_area 必须大于零")
        if all(_is_finite_number(value) for value in (
                original_area, reachable_area, excluded_area)):
            area_tolerance = 1e-6 * max(1.0, original_area)
            if abs(reachable_area + excluded_area - original_area) > area_tolerance:
                errors.append("cspace 面积不守恒")
        if all(_is_finite_number(value) for value in (
                original_area, excluded_area, excluded_ratio)) and original_area > 0.0:
            expected_ratio = excluded_area / original_area
            if abs(excluded_ratio - expected_ratio) > 1e-6:
                errors.append("cspace.excluded_ratio 与面积不一致")
        for field_name, value in (
                ("excluded_ratio", excluded_ratio), ("max_ratio", max_ratio)):
            if _is_finite_number(value) and not 0.0 <= value <= 1.0:
                errors.append(f"cspace.{field_name}={value} 超出范围 [0, 1]")
        if _is_finite_number(excluded_ratio) and _is_finite_number(max_ratio) and (
                excluded_ratio > max_ratio + 1e-9):
            errors.append("cspace.excluded_ratio 超过 cspace.max_ratio")
        if cspace.get("gate") != "PASS":
            errors.append("cspace.gate 未通过")

        component_count = cspace.get("component_count")
        if (not isinstance(component_count, int) or
                isinstance(component_count, bool) or component_count < 1):
            errors.append("cspace.component_count 必须是正整数")
        requires_repositioning = cspace.get("requires_repositioning")
        if not isinstance(requires_repositioning, bool):
            errors.append("cspace.requires_repositioning 必须是布尔值")
        elif isinstance(component_count, int) and not isinstance(
                component_count, bool) and (
                requires_repositioning != (component_count > 1)):
            errors.append(
                "cspace.requires_repositioning 与 component_count 不一致")
    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description="校验单场景 F2C 批测结果")
    parser.add_argument("report", help="场景的 *_data.json 文件")
    parser.add_argument(
        "--coverage-threshold",
        type=float,
        default=0.99,
        help="覆盖率门槛，使用 0 到 1 的比例（默认 0.99）",
    )
    args = parser.parse_args(argv)

    report = None
    report_path = Path(args.report)
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        errors = validate_report(report, args.coverage_threshold)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        errors = [f"无法读取或校验报告：{error}"]

    scenario = report_path.stem
    if isinstance(report, dict):
        scenario = report.get("scenario") or scenario
    if errors:
        print(f"  GATE FAIL [{scenario}]")
        for error in errors:
            print(f"    - {error}")
        return 1

    cspace = report.get("cspace", {})
    if cspace.get("component_count", 1) > 1:
        coverage_rate = min(
            evaluation["effective_coverage_rate"]
            for evaluation in report["component_evals"])
    else:
        coverage_rate = report["eval"]["effective_coverage_rate"]
    print(f"  GATE PASS [{scenario}]: coverage={coverage_rate:.2f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
