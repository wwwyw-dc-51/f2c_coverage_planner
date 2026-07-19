# F2C Coverage Planner v9.11 发布说明

> **发布日期**: 2026-07-20
> **分支**: `release/v9.11-hole-fix-and-s8-rename`
> **基准**: batch_0720_0415

---

## 版本概述

v9.11 是一个**稳定修复版本**，在 v9.7 基础上累积了 C-space 退役、空隙分类器、Phase 4A 加回、filter/merge 顺序修复、S8 场景重命名，以及关键的孔洞穿越硬约束修复。

## 核心改动

### 算法

| 改动 | 说明 |
|:---|:---|
| **C-space 退役** (v9.8) | 由空隙分类器替代，区分"物理不可达"和"算法可改进" |
| **Phase 4A 加回** (v9.8) | sweep alignment + adaptive headland + slant detection |
| **空隙分类器** (v9.8) | 连通分量 + 最大内切宽度，修正覆盖率 = covered / (target - unreachable) |
| **filter/merge 顺序 swap** (v9.9) | filterTinyCells 移到 mergeCellsWithSimilarDirection 之后 |
| **Per-cell 离群 veto** (v9.11) | 全局角度优化 + 逐 cell 离群检测（2.0x + 3 阈值） |
| **孔洞穿越硬约束** (v9.11) | `greedyCellOrder` 穿洞惩罚 +1000 → +1e9 |

### 场景

| 改动 | 说明 |
|:---|:---|
| **S7 工厂车间** | 新增 30m×20m 10 孔洞复杂场景 |
| **notched → S8** | 命名一致性重命名 |

### 基础设施

| 改动 | 说明 |
|:---|:---|
| 批测门控加固 | 99% 覆盖门槛参与退出判定，文件缺失检测 |
| 双覆盖率报告 | 原始覆盖率 + 修正覆盖率 |
| Vis JSON 修复 | cells 使用 decomposition_cells |
| 渲染修复 | 包围盒裁剪、alpha 调整 |

---

## 基准数据 (batch_0720_0415)

参数: `robot_width=0.75`, `coverage_width=0.90`, `swath_angle_optimization=true`, `use_sweep_decomp=true`

| 场景 | 覆盖率 | 得分 | 重叠率 | hole_crossings |
|:---|:------:|:----:|:-----:|:---:|
| S1 矩形 20×15m | 100.00% | 96.5 | 7.8% | 0 |
| S2 L 形 | 100.00% | 94.8 | 7.5% | 0 |
| S3 含孔洞 | 100.00% | 74.5 | 14.5% | 0 |
| S4 窄走廊 | 100.00% | 89.1 | 29.1% | 0 |
| S5 不规则 | 100.00% | 84.0 | 16.3% | 0 |
| S6 多区域 | 100.00% | 72.1 | 29.6% | 0 |
| **S7 工厂车间** | **99.81%** | **62.2** | 44.9% | **0** |
| S8 缺口+中心孔 | 100.00% | 78.9 | 24.0% | 0 |
| **平均** | **99.98%** | **81.5** | — | — |

---

## 已知问题

| 问题 | 严重度 | 说明 |
|:---|:---:|:---|
| S7 孔洞贴边路径 | P1 | swath 端点紧贴孔洞边界，连接段沿孔洞走 |
| S7 高重叠率 44.9% | P1 | per-cell veto 导致 cell 间角度不一致 |
| S3/S5/S6 轻微退化 | P2 | 边角度收集策略对特定场景不是最优 |
| coverage_width 未冻结 | P1 | 待直线清扫痕迹实测确认 |

---

## 部署

### 从 GitHub 获取

```bash
git clone https://github.com/wwwyw-dc-51/f2c_coverage_planner.git
cd f2c_coverage_planner
git checkout release/v9.11-hole-fix-and-s8-rename
```

### 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yingshi_robot --symlink-install
```

### 跑批测

```bash
bash sync_and_build.sh --batch
```

详细部署步骤见 `SETUP_GUIDE.md`。

---

## 文件清单

| 文件 | 说明 |
|:---|:---|
| `README.md` | 项目总览 + 基准数据 |
| `SETUP_GUIDE.md` | 双端部署指南 |
| `docs/F2C_技术文档.md` | 完整技术文档 |
| `docs/daily_logs/2026-07-20.md` | v9.11 开发日志 |
| `scripts/batch_test_v2.sh` | 8 场景批量测试 |
| `scripts/sync_and_build.sh` | 一键工作流 |

---

> 🤖 Generated with [Claude Code](https://claude.com/claude-code)
