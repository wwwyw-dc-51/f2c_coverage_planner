# F2C Coverage Planner — Claude 项目指南

## 项目概述
ROS2 Humble + Fields2Cover 扫地机器人全覆盖路径规划。纯 C++ PlannerCore 管线。

> **当前版本**: v10.0 十全十美 (2026-07-23) | **测试**: 21/21 GATE PASS

## 快速开始

```bash
# 连接 VM
ssh dc@192.168.83.129

# 同步 + 编译
cd ~/f2c_coverage_planner
bash scripts/sync_and_build.sh

# 快速开发测试（3-5 个关键场景，无渲染）
bash scripts/quick_dev_test.sh S3 S7 N2 N4 N12

# 全量 21 场景测试（含可视化渲染）
bash scripts/batch_test_v2.sh
```

## 架构

```
PlannerCore::plan()           ← 唯一管线（纯 C++）
  ├── rectilinearDecompose()  ← sweep 扫描线分解
  ├── mergeCellsWithSimilarDirection() ← 第一轮合并（v9.11 顶点邻近+质心孔洞保护）
  ├── mergeAdjacentSweepStrips() ← 第二轮合并（同 x-span+垂直相邻，interior ring 把关）
  ├── generateSwathsForAllCells() ← swath 生成
  ├── greedyCellOrder → genRoute → holeRepair → pathPlanning
  └── 评估 + 可视化
```

## 关键文件

| 文件 | 职责 |
|------|------|
| `src/yingshi_robot/src/planner_core.cpp` | PlannerCore 主流程 |
| `src/yingshi_robot/src/decomposer.cpp` | 分解 + 合并（mergeCellsWithSimilarDirection, mergeAdjacentSweepStrips） |
| `src/yingshi_robot/src/swath_generator.cpp` | Swath 生成 + 角度优化 |
| `src/yingshi_robot/src/boundary_filler.cpp` | 边界补线 + adjustSwathEndpointsAdaptive |
| `src/yingshi_robot/src/polygon_planner_node.cpp` | ROS2 节点（调 PlannerCore，legacy merge 已禁用） |

## 分支说明

| 分支 | 用途 |
|------|------|
| `master` | 稳定主线（PlannerCore only） |
| `legacy-dual-pipeline` | 旧双管线存档（GitHub 备份） |
| `main-stable` | v9.11 + cell 合并 + 可见图路由（有穿洞问题） |
| `feat/param-decouple-0721` | robot_width 参数解耦实验 |

## 当前状态 (2026-07-21)

- **合并策略**: 两轮安全合并（v9.11 + x-span），interior ring 安全网
- **管线**: PlannerCore only，旧 merge 已禁用
- **21 场景**: 20/21 PASS（N3_dense 98.10% 低于 99% 门槛）
- **已知问题**: robot_width 参数更新待解决（feat/param-decouple-0721），adjustSwathEndpointsAdaptive 待正确接入

## 测试结果

`test_results/` 目录（gitignored）:
- `batch_0721_1400/` — 无合并基线 (21/21 PASS)
- `batch_final/` — 两轮合并最新结果 (20/21 PASS)
- `batch_safe_v2/` — 共享边检测翻车版
