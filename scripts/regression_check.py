#!/usr/bin/env python3
"""
F2C regression check — compare batch results against golden baseline.

Usage:
  python scripts/regression_check.py <test_dir> [golden_dir]
"""
import sys
import io

# Windows GBK consoles cannot render Unicode checkmarks.
# Force UTF-8 on all platforms; wrap stdout for safety.
if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(
        sys.stdout.buffer, encoding='utf-8', errors='replace')
if sys.stderr.encoding != 'utf-8':
    sys.stderr = io.TextIOWrapper(
        sys.stderr.buffer, encoding='utf-8', errors='replace')

# Default golden dir, tolerances, exit codes documented in --help / main() docstring.

import json
import os
import sys
from pathlib import Path

SCENARIOS = ["S1", "S2", "S3", "S4", "S5", "S6", "notched"]

# 容忍阈值 — 在此范围内的变化视为噪声，不报告为回退
TOLERANCE = {
    "coverage_rate_pct": 0.15,       # 覆盖率允许降 0.15%
    "single_score": 2.0,             # 得分允许降 2.0
    "turn_count_abs": 3,             # 转弯允许增加 3
    "overlap_rate_pct": 5.0,         # 重叠率绝对值允许增加 5.0pp
    "total_distance_rel": 0.10,      # 路径长相对增加允许 10%
    "uncovered_area_abs": 0.50,      # 未覆盖面积允许增加 0.50 m²
}

COLORS = {"red": "\033[91m", "green": "\033[92m", "yellow": "\033[93m",
          "reset": "\033[0m", "bold": "\033[1m"}


def load_metrics(run_dir: str) -> dict:
    """从批测目录加载各场景指标。"""
    metrics = {}
    for s in SCENARIOS:
        data_path = Path(run_dir) / f"{s}_data.json"
        if not data_path.exists():
            print(f"{COLORS['red']}  MISSING: {data_path}{COLORS['reset']}")
            metrics[s] = None
            continue
        with open(data_path) as f:
            d = json.load(f)
        e = d["eval"]
        metrics[s] = {
            "coverage_rate": e["coverage_rate"],
            "single_score": e["single_score"],
            "turn_count": e["turn_count"],
            "overlap_rate": e["overlap_rate"],
            "total_distance": e["total_distance"],
            "work_ratio": e["work_ratio"],
            "uncovered_area": e["uncovered_area"],
            "path_points": len(d.get("path", [])),
            "cells": len(d.get("cells", [])),
            "connections": len(d.get("connections", [])),
        }
    return metrics


def check_scenario(name: str, test: dict, golden: dict) -> list:
    """对比单个场景，返回回退列表。"""
    failures = []

    if test is None:
        failures.append("数据文件缺失")
        return failures
    if golden is None:
        failures.append("黄金基线缺失")
        return failures

    # 覆盖率 — 只能降不能升太多？不是，只检查下降
    cov_delta = test["coverage_rate"] - golden["coverage_rate"]
    if cov_delta < -TOLERANCE["coverage_rate_pct"]:
        failures.append(
            f"覆盖率 {golden['coverage_rate']:.2f}→{test['coverage_rate']:.2f}% "
            f"({cov_delta:+.2f}%, 阈值 {-TOLERANCE['coverage_rate_pct']:.2f}%)")

    # 得分
    score_delta = test["single_score"] - golden["single_score"]
    if score_delta < -TOLERANCE["single_score"]:
        failures.append(
            f"得分 {golden['single_score']:.1f}→{test['single_score']:.1f} "
            f"({score_delta:+.1f}, 阈值 {-TOLERANCE['single_score']:.1f})")

    # 转弯数
    turn_delta = test["turn_count"] - golden["turn_count"]
    if turn_delta > TOLERANCE["turn_count_abs"]:
        failures.append(
            f"转弯 {golden['turn_count']}→{test['turn_count']} "
            f"({turn_delta:+d}, 阈值 +{TOLERANCE['turn_count_abs']})")

    # 重叠率
    overlap_delta = test["overlap_rate"] - golden["overlap_rate"]
    if overlap_delta > TOLERANCE["overlap_rate_pct"]:
        failures.append(
            f"重叠率 {golden['overlap_rate']:.1f}→{test['overlap_rate']:.1f}% "
            f"({overlap_delta:+.1f}pp, 阈值 +{TOLERANCE['overlap_rate_pct']:.1f}pp)")

    # 路径长 — 相对增加
    if golden["total_distance"] > 0.1:
        path_rel = (test["total_distance"] - golden["total_distance"]) / golden["total_distance"]
        if path_rel > TOLERANCE["total_distance_rel"]:
            failures.append(
                f"路径长 {golden['total_distance']:.1f}→{test['total_distance']:.1f}m "
                f"({path_rel:+.1%}, 阈值 +{TOLERANCE['total_distance_rel']:.0%})")

    # 未覆盖面积
    uncovered_delta = test["uncovered_area"] - golden["uncovered_area"]
    if uncovered_delta > TOLERANCE["uncovered_area_abs"]:
        failures.append(
            f"未覆盖 {golden['uncovered_area']:.2f}→{test['uncovered_area']:.2f}m² "
            f"({uncovered_delta:+.2f}m², 阈值 +{TOLERANCE['uncovered_area_abs']:.2f}m²)")

    return failures


def print_report(test_metrics: dict, golden_metrics: dict, failures_by_scene: dict):
    """打印人类可读的回归报告。"""
    print()
    print(f"{COLORS['bold']}{'='*70}{COLORS['reset']}")
    print(f"{COLORS['bold']}  F2C 回归检查报告{COLORS['reset']}")
    print(f"{COLORS['bold']}{'='*70}{COLORS['reset']}")
    print()

    # 表头
    header = f" {'场景':<8} {'覆盖率':>8} {'得分':>6} {'转弯':>4} {'重叠率':>7} {'路径长':>8} {'未覆盖':>7}  判定"
    print(header)
    print("-" * 78)

    total_failures = 0
    for s in SCENARIOS:
        t = test_metrics.get(s)
        g = golden_metrics.get(s)
        fails = failures_by_scene.get(s, [])

        if t is None:
            print(f" {s:<8} {'—':>8} {'—':>6} {'—':>4} {'—':>7} {'—':>8} {'—':>7}  {COLORS['red']}数据缺失{COLORS['reset']}")
            total_failures += 1
            continue

        cov_s = f"{t['coverage_rate']:.2f}%"
        score_s = f"{t['single_score']:.1f}"
        turn_s = str(t['turn_count'])
        overlap_s = f"{t['overlap_rate']:.1f}%"
        path_s = f"{t['total_distance']:.1f}m"
        uncov_s = f"{t['uncovered_area']:.2f}"

        if not fails:
            flag = f"{COLORS['green']}✓ 通过{COLORS['reset']}"
        else:
            flag = f"{COLORS['red']}✗ {len(fails)}项回退{COLORS['reset']}"
            total_failures += 1

        print(f" {s:<8} {cov_s:>8} {score_s:>6} {turn_s:>4} {overlap_s:>7} {path_s:>8} {uncov_s:>7}  {flag}")

        # 打印具体回退项
        for fail in fails:
            print(f"         {COLORS['yellow']}↳ {fail}{COLORS['reset']}")

    print()
    if total_failures == 0:
        print(f"{COLORS['green']}{COLORS['bold']}  全部通过 — 零回退 ✓{COLORS['reset']}")
    else:
        print(f"{COLORS['red']}{COLORS['bold']}  {total_failures} 个场景有回退 ✗{COLORS['reset']}")

    print()
    return total_failures


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    test_dir = sys.argv[1]
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent
    golden_dir = sys.argv[2] if len(sys.argv) > 2 else str(
        project_root / "test" / "golden" / "batch_0717_1310")

    if not Path(test_dir).exists():
        print(f"错误: 测试目录不存在: {test_dir}")
        sys.exit(2)
    if not Path(golden_dir).exists():
        print(f"错误: 黄金基线目录不存在: {golden_dir}")
        sys.exit(2)

    print(f"测试目录: {test_dir}")
    print(f"黄金基线: {golden_dir}")

    test_metrics = load_metrics(test_dir)
    golden_metrics = load_metrics(golden_dir)

    failures_by_scene = {}
    for s in SCENARIOS:
        fails = check_scenario(
            s,
            test_metrics.get(s),
            golden_metrics.get(s))
        if fails:
            failures_by_scene[s] = fails

    total = print_report(test_metrics, golden_metrics, failures_by_scene)
    sys.exit(0 if total == 0 else 1)


if __name__ == "__main__":
    main()
