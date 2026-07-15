#!/usr/bin/env python3
"""
F2C 全覆盖规划 — Phase 3 7场景批量测试
=========================================
测试矩阵：
  - 7 场景: notched_L, S1, S2, S3, S4, S5, S6
  - 2 模式: custom (boustrophedon), snake
  - 参数固化: robot=0.95, cov=0.45, sweep=true, merge=60°, RDP=0.05, turn=direct

输出：
  - test_results/batch_0713_phase3/
    {scenario}_{mode}_path.json     — 路径点
    {scenario}_{mode}_eval.json     — 评估指标
    {scenario}_comparison.png       — custom vs snake 热力图对比
    summary.json                    — 汇总对比表

用法：
  python batch_test_phase3.py                    # 全部 14 个组合
  python batch_test_phase3.py --scenario S3      # 单个场景
  python batch_test_phase3.py --mode snake       # 仅 snake 模式
"""

import argparse
import json
import os
import subprocess
import sys
import time
import yaml
from datetime import datetime
from pathlib import Path

# ── 路径配置 ──
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent  # src/yingshi_robot/scripts → workspace
TEST_POLYGONS_DIR = SCRIPT_DIR.parent / "test_polygons"
CONFIG_AREAS_DIR = SCRIPT_DIR.parent / "config" / "f2c_areas"
OUTPUT_DIR = WORKSPACE_DIR / "test_results" / f"batch_0713_phase3"

# ── 场景定义 ──
SCENARIOS = {
    "notched_L": CONFIG_AREAS_DIR / "notched_10m_with_center_hole.yaml",
    "S1": TEST_POLYGONS_DIR / "S1_S1_convex_rect.yaml",
    "S2": TEST_POLYGONS_DIR / "S2_S2_L_shaped.yaml",
    "S3": TEST_POLYGONS_DIR / "S3_S3_with_holes.yaml",
    "S4": TEST_POLYGONS_DIR / "S4_S4_narrow_corridor.yaml",
    "S5": TEST_POLYGONS_DIR / "S5_S5_irregular.yaml",
    "S6": TEST_POLYGONS_DIR / "S6_S6_multi_region.yaml",
}

# ── 固化参数 ──
FIXED_PARAMS = {
    "robot_width": 0.95,
    "coverage_width": 0.45,
    "use_sweep_decomp": True,
    "merge_angle_threshold": 60.0,
    "path_simplify_tolerance": 0.05,
    "turn_planner_type": "direct",
    "swath_angle_optimization": True,
    "decomposition_enabled": True,
    "filter_tiny_cells": True,
    "path_simplify_enabled": True,
    "swath_overlap_ratio": 0.03,
    "use_optimized_planner": True,
    "eval_enable_report": True,
    "eval_use_grid_method": True,
    "eval_grid_resolution": 0.05,
    "min_swath_length": 0.5,
    "boundary_type": "auto",  # 从 YAML 读
}


def make_ros_params(scenario_path, mode):
    """构建 ROS2 参数字符串"""
    params = dict(FIXED_PARAMS)
    params["swath_order_type"] = mode  # "boustrophedon" or "snake"

    # 从 YAML 读取 boundary_type
    with open(scenario_path) as f:
        data = yaml.safe_load(f)
    params["boundary_type"] = data.get("boundary_type", "closed")

    # 构建命令行参数
    args = []
    for key, val in params.items():
        if isinstance(val, bool):
            args.append(f"{key}:={str(val).lower()}")
        elif isinstance(val, float):
            args.append(f"{key}:={val}")
        else:
            args.append(f"{key}:={val}")
    return args


def run_scenario(scenario_name, scenario_path, mode, dry_run=False):
    """运行单个场景测试，返回评估结果"""
    print(f"\n{'='*60}")
    print(f"  {scenario_name} — {mode} mode")
    print(f"{'='*60}")

    # 构建命令（使用现有的 run_f2c_optimized.sh）
    ros_params = make_ros_params(scenario_path, mode)
    param_str = " ".join(f"-p {p}" for p in ros_params)

    # 通过 ROS2 run 直接运行（非交互模式）
    cmd = [
        "ros2", "run", "yingshi_robot", "polygon_planner_node",
        "--ros-args",
    ] + [item for p in ros_params for item in ("-p", p)]

    if dry_run:
        print(f"  [DRY RUN] {' '.join(cmd)}")
        return None

    # TODO: 实际运行逻辑 — 需要 ROS2 环境
    # 此处提供框架，实际运行在目标机器上执行
    print(f"  CMD: {' '.join(cmd)}")
    return None


def generate_report(results):
    """生成汇总报告"""
    report = {
        "timestamp": datetime.now().isoformat(),
        "params": FIXED_PARAMS,
        "scenarios": {},
        "summary": {
            "total_scenarios": len(results),
            "custom_avg_coverage": 0.0,
            "snake_avg_coverage": 0.0,
            "custom_avg_score": 0.0,
            "snake_avg_score": 0.0,
        },
    }

    custom_covs, snake_covs = [], []
    custom_scores, snake_scores = [], []

    for r in results:
        key = f"{r['scenario']}_{r['mode']}"
        report["scenarios"][key] = r

        if r["mode"] == "custom":
            custom_covs.append(r.get("coverage", 0))
            custom_scores.append(r.get("score", 0))
        else:
            snake_covs.append(r.get("coverage", 0))
            snake_scores.append(r.get("score", 0))

    if custom_covs:
        report["summary"]["custom_avg_coverage"] = sum(custom_covs) / len(custom_covs)
        report["summary"]["custom_avg_score"] = sum(custom_scores) / len(custom_scores)
    if snake_covs:
        report["summary"]["snake_avg_coverage"] = sum(snake_covs) / len(snake_covs)
        report["summary"]["snake_avg_score"] = sum(snake_scores) / len(snake_scores)

    return report


def main():
    parser = argparse.ArgumentParser(description="F2C Phase 3 批量测试")
    parser.add_argument("--scenario", "-s", type=str, help="单个场景")
    parser.add_argument("--mode", "-m", type=str, choices=["custom", "snake"],
                        help="单个模式")
    parser.add_argument("--dry-run", action="store_true", help="仅打印命令不执行")
    args = parser.parse_args()

    # 过滤场景
    if args.scenario:
        scenarios = {args.scenario: SCENARIOS[args.scenario]}
    else:
        scenarios = SCENARIOS

    modes = [args.mode] if args.mode else ["custom", "snake"]

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"F2C Phase 3 批量测试")
    print(f"  场景: {list(scenarios.keys())}")
    print(f"  模式: {modes}")
    print(f"  输出: {OUTPUT_DIR}")
    print(f"  参数: robot=0.95 cov=0.45 sweep=true merge=60° RDP=0.05 turn=direct")

    results = []
    for name, path in scenarios.items():
        if not path.exists():
            print(f"  ⚠ 场景文件缺失: {path}")
            continue
        for mode in modes:
            result = run_scenario(name, path, mode, dry_run=args.dry_run)
            if result:
                results.append(result)

    # 生成报告
    report = generate_report(results)
    report_path = OUTPUT_DIR / "summary.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n{'='*60}")
    print(f"  测试完成！报告: {report_path}")
    print(f"  Custom 平均覆盖率: {report['summary']['custom_avg_coverage']:.1%}")
    print(f"  Snake  平均覆盖率: {report['summary']['snake_avg_coverage']:.1%}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
