#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F2C 全覆盖路径规划 — 测试场景生成器

生成 6 类不同复杂度的测试多边形 YAML 文件，
格式与 Fields2Cover demo 的 YAML 完全兼容。

用法：
  python generate_test_scenes.py                      # 全部 6 个场景
  python generate_test_scenes.py --scenario S1        # 只生成 S1
  python generate_test_scenes.py --scenario S1,S3,S5  # 生成指定场景
  python generate_test_scenes.py --output-dir ./my_tests  # 指定输出目录
"""

import argparse
import os
import sys
import math
from pathlib import Path

# ============================================================================
# 场景定义
# ============================================================================

SCENARIOS = {}


def register(name, description):
    """装饰器：注册场景定义函数"""
    def decorator(fn):
        SCENARIOS[name] = {"fn": fn, "desc": description}
        return fn
    return decorator


# ── S1: 简单凸多边形 ──
@register("S1", "简单凸多边形 — 20m×15m 矩形，无孔洞，基准性能测试")
def S1_convex_rect():
    return {
        "polygon": [
            [0.0, 0.0],
            [20.0, 0.0],
            [20.0, 15.0],
            [0.0, 15.0],
        ],
        "holes": [],
        "boundary_type": "closed",   # 闭合硬边界 — 室内房间，swath 端点内缩留安全距离
    }


# ── S2: L 形凹多边形 ──
@register("S2", "L 形凹多边形 — 25m 尺度，无孔洞，测试区域分解合理性")
def S2_L_shaped():
    return {
        "polygon": [
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 12.0],
            [25.0, 12.0],
            [25.0, 25.0],
            [0.0, 25.0],
        ],
        "holes": [],
        "boundary_type": "closed",   # 闭合硬边界 — L 形房间
    }


# ── S3: 带孔洞多边形 ──
@register("S3", "带孔洞 — 25m×25m 含 2 个不同大小的矩形障碍物")
def S3_with_holes():
    return {
        "polygon": [
            [0.0, 0.0],
            [25.0, 0.0],
            [25.0, 25.0],
            [0.0, 25.0],
        ],
        "holes": [
            # 大孔洞：10m×10m 居中偏右
            [
                [13.0, 7.5],
                [23.0, 7.5],
                [23.0, 17.5],
                [13.0, 17.5],
            ],
            # 小孔洞：3m×3m 左下角
            [
                [2.0, 2.0],
                [5.0, 2.0],
                [5.0, 5.0],
                [2.0, 5.0],
            ],
        ],
        "boundary_type": "closed",   # 闭合硬边界 — 有障碍物的房间
    }


# ── S4: 狭长走廊 ──
@register("S4", "狭长走廊 — 2m×20m，Swath 方向高度敏感")
def S4_narrow_corridor():
    return {
        "polygon": [
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 20.0],
            [0.0, 20.0],
        ],
        "holes": [],
        "boundary_type": "closed",   # 闭合硬边界 — 狭长走廊，边界不可越
    }


# ── S5: 复杂不规则形 ──
@register("S5", "复杂不规则 — 模拟真实房间布局（凹凸交错）")
def S5_irregular():
    # 模拟一个有凹凸墙面的真实房间：外框 20m×16m，带凹口和凸角
    return {
        "polygon": [
            [0.0, 0.0],
            [8.0, 0.0],
            [8.0, 4.0],    # 凹口底部
            [12.0, 4.0],
            [12.0, 0.0],   # 凹口顶部
            [20.0, 0.0],
            [20.0, 10.0],
            [16.0, 10.0],
            [16.0, 16.0],  # 凸出
            [12.0, 16.0],
            [12.0, 10.0],  # 凸出结束
            [8.0, 10.0],
            [8.0, 14.0],   # 另一个凸出
            [4.0, 14.0],
            [4.0, 10.0],
            [0.0, 10.0],
        ],
        "holes": [],
        "boundary_type": "open",     # 开放软边界 — 复杂不规则形，允许边角外伸换覆盖率
    }


# ── S6: 多 disconnected 区域 ──
@register("S6", "多断开区域 — 全高走廊分隔出 3 个独立矩形分区")
def S6_multi_region():
    # 30m×20m 的外框，用 2 个全高垂直走廊（hole）完全切出 3 个独立区域：
    #   - 左区: x=[0,8],  y=[0,20]  (8m×20m)
    #   - 中区: x=[12,18], y=[0,20]  (6m×20m)
    #   - 右区: x=[22,30], y=[0,20]  (8m×20m)
    # 走廊宽度 4m，确保 headland 处理后三个区域完全断开
    return {
        "polygon": [
            [0.0, 0.0],
            [30.0, 0.0],
            [30.0, 20.0],
            [0.0, 20.0],
        ],
        "holes": [
            # 走廊 1: 分隔左区和中区，y 从 0 到 20（全高）
            [
                [8.0, 0.0],
                [12.0, 0.0],
                [12.0, 20.0],
                [8.0, 20.0],
            ],
            # 走廊 2: 分隔中区和右区，y 从 0 到 20（全高）
            [
                [18.0, 0.0],
                [22.0, 0.0],
                [22.0, 20.0],
                [18.0, 20.0],
            ],
        ],
        "boundary_type": "open",     # 开放软边界 — 多断开区域，允许外伸牺牲重叠换覆盖率
    }


# ============================================================================
# YAML 输出
# ============================================================================

def format_yaml(scene_data, scenario_name, description):
    """将场景数据格式化为 YAML 字符串"""
    lines = []
    lines.append(f"# F2C 测试场景 {scenario_name}")
    lines.append(f"# {description}")
    lines.append(f"# 自动生成 — 请勿手动编辑")
    lines.append(f"#")
    lines.append(f"polygon:")

    for point in scene_data["polygon"]:
        lines.append(f"  - [{point[0]:.1f}, {point[1]:.1f}]")

    if scene_data.get("holes"):
        lines.append(f"")
        lines.append(f"holes:")
        for hole_idx, hole in enumerate(scene_data["holes"]):
            lines.append(f"  - - [{hole[0][0]:.1f}, {hole[0][1]:.1f}]")
            for point in hole[1:]:
                lines.append(f"    - [{point[0]:.1f}, {point[1]:.1f}]")
    else:
        lines.append(f"")
        lines.append(f"holes: []")

    # 边界类型（闭合/开放）
    boundary_type = scene_data.get("boundary_type", "closed")
    lines.append(f"")
    lines.append(f"boundary_type: {boundary_type}")

    lines.append("")
    return "\n".join(lines)


def generate_scenes(scenario_ids, output_dir):
    """生成指定场景的 YAML 文件"""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    generated = []
    for sid in scenario_ids:
        if sid not in SCENARIOS:
            print(f"⚠️  未知场景: {sid}，跳过")
            continue

        info = SCENARIOS[sid]
        scene_data = info["fn"]()
        yaml_content = format_yaml(scene_data, sid, info["desc"])

        filename = f"{sid}_{info['fn'].__name__}.yaml"
        filepath = output_dir / filename
        filepath.write_text(yaml_content, encoding="utf-8")

        polygon_pts = len(scene_data["polygon"])
        hole_count = len(scene_data.get("holes", []))
        print(f"✅ {sid}: {filename}  ({polygon_pts} 个多边形顶点, {hole_count} 个孔洞)")
        generated.append(str(filepath))

    return generated


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="F2C 全覆盖路径规划 — 测试场景生成器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                             生成全部 6 个场景
  %(prog)s --scenario S1               只生成 S1
  %(prog)s --scenario S1,S3,S5         生成指定场景
  %(prog)s --output-dir ./my_tests     输出到自定义目录
  %(prog)s --list                      列出所有场景
        """,
    )

    parser.add_argument(
        "--scenario", "-s",
        type=str,
        default="all",
        help="要生成的场景 ID，逗号分隔（默认: all）",
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=None,
        help="输出目录（默认: 脚本所在目录的 ../test_polygons/）",
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="列出所有可用场景及其描述",
    )

    args = parser.parse_args()

    # --list 模式
    if args.list:
        print("\n📋 可用测试场景:\n")
        for sid in sorted(SCENARIOS.keys()):
            print(f"  {sid}: {SCENARIOS[sid]['desc']}")
        print()
        return

    # 确定输出目录
    if args.output_dir:
        output_dir = args.output_dir
    else:
        script_dir = Path(__file__).resolve().parent
        output_dir = script_dir.parent / "test_polygons"

    # 确定要生成的场景
    if args.scenario.lower() == "all":
        scenario_ids = sorted(SCENARIOS.keys())
    else:
        scenario_ids = [s.strip() for s in args.scenario.split(",")]

    print(f"\n🔧 F2C 测试场景生成器")
    print(f"   输出目录: {output_dir}")
    print(f"   场景数量: {len(scenario_ids)}\n")

    generated = generate_scenes(scenario_ids, output_dir)

    print(f"\n📊 生成完成: {len(generated)}/{len(scenario_ids)} 个场景")
    if generated:
        print(f"   文件列表:")
        for f in generated:
            print(f"     • {f}")
    print()


if __name__ == "__main__":
    main()
