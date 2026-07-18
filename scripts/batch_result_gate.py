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
    "single_score",
    "uncovered_area",
    "total_distance",
    "work_ratio",
    "turn_count",
    "overlap_rate",
    "planning_time_ms",
    "net_area",
)
BOUNDED_EVAL_METRICS = {
    "coverage_rate": (0.0, 100.0),
    "single_score": (0.0, 100.0),
}
NON_NEGATIVE_EVAL_METRICS = (
    "uncovered_area",
    "total_distance",
    "work_ratio",
    "overlap_rate",
    "planning_time_ms",
)


def _is_finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


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

    path = report.get("path")
    if not isinstance(path, list) or len(path) < 2:
        errors.append("path 缺失或不足两个点")
    swaths = report.get("swaths")
    if not isinstance(swaths, list) or not swaths:
        errors.append("swaths 缺失或为空")

    evaluation = report.get("eval")
    if not isinstance(evaluation, dict):
        evaluation = {}
        errors.append("eval 缺失或不是对象")
    for metric_name in REQUIRED_EVAL_METRICS:
        if not _is_finite_number(evaluation.get(metric_name)):
            errors.append(f"eval.{metric_name} 缺失或不是有限数值")

    for metric_name, (minimum, maximum) in BOUNDED_EVAL_METRICS.items():
        value = evaluation.get(metric_name)
        if _is_finite_number(value) and not minimum <= value <= maximum:
            errors.append(
                f"eval.{metric_name}={value} 超出范围 [{minimum}, {maximum}]")
    for metric_name in NON_NEGATIVE_EVAL_METRICS:
        value = evaluation.get(metric_name)
        if _is_finite_number(value) and value < 0.0:
            errors.append(f"eval.{metric_name}={value} 不能为负数")

    turn_count = evaluation.get("turn_count")
    if _is_finite_number(turn_count) and (
            not isinstance(turn_count, int) or turn_count < 0):
        errors.append("eval.turn_count 必须是非负整数")
    net_area = evaluation.get("net_area")
    if _is_finite_number(net_area) and net_area <= 0.0:
        errors.append("eval.net_area 必须大于零")

    coverage_rate = evaluation.get("coverage_rate")
    if _is_finite_number(coverage_rate) and (
            coverage_rate + 1e-9 < coverage_threshold * 100.0):
        errors.append(
            f"coverage_rate={coverage_rate:.2f}% 低于门槛 "
            f"{coverage_threshold * 100.0:.2f}%")
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

    coverage_rate = report["eval"]["coverage_rate"]
    print(f"  GATE PASS [{scenario}]: coverage={coverage_rate:.2f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
