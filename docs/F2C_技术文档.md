# F2C 全覆盖路径规划系统 — 技术文档

> **版本**: v9.11 穿洞修复版 (2026-07-20)
> **编译状态**: 0 error 0 warning | **基准**: 8 场景零回归，hole_crossings=0

---

## 目录

1. [项目概述](#1-项目概述)
2. [算法选型](#2-算法选型)
3. [核心逻辑](#3-核心逻辑)
4. [参数说明](#4-参数说明)
5. [评估体系](#5-评估体系)
6. [已知问题与技术债](#6-已知问题与技术债)
7. [运行方式](#7-运行方式)

---

## 1. 项目概述

基于 **Fields2Cover (F2C)** 开源库 + **ROS2 Humble** 的扫地机器人全覆盖路径规划系统。

输入任意多边形区域（可含孔洞障碍物），输出完整的覆盖路径。

### 1.1 机器人配置

| 参数 | 值 | 说明 |
|------|------|------|
| 车体宽度 | 0.75 m | robot_width（2026-07-16 实车测量） |
| 覆盖宽度 | 约 0.90 m | coverage_width（两个直径 0.45 m 的软质清洁轮组合名义宽度，待痕迹实测） |
| 最小转弯半径 | ~0 m | 差速驱动，可原地转向 |
| 最大允许曲率 | 0.3 | max_diff_curv |

> 历史 demo 和 `batch_0716_1331` 使用 `robot_width=0.95`、`coverage_width=0.45`。这些值保留用于解释历史报告，不再作为产品物理参数推荐值。

### 1.2 v9 模块化架构

```
polygon_planner_node.cpp (3930行)  ← 主节点 + ROS2 通信
    ├── 调用 → decomposer.cpp          (340行) Sweep 扫描线分解 + 环简化
    ├── 调用 → swath_generator.cpp     (274行) Swath 优化 + 端点调整 + 几何变换
    ├── 调用 → boundary_filler.cpp     (354行) 边界/孔洞补刀 (含 getLinesInside)
    ├── 调用 → path_planner.cpp        (202行) RDP 简化 + 掉头检测
    └── 引用 → coverage_evaluator.hpp  (700行) 网格覆盖评估
```

### 1.3 系统流程

```
YAML 多边形 → ROS2 Topic → polygon_planner_node (C++)
                                │
                ┌───────────────┼───────────────┐
                │               │               │
          分解(decomposer)  Swath生成      路径规划(path_planner)
          Sweep扫描线      多角度优化      TSP genRoute
          全宽条带Cells    fillBoundaryGaps  Direct/RDP
                │               │               │
                └───────────────┼───────────────┘
                                │
                    评估(coverage_evaluator)
                    网格采样 → 覆盖率 + 得分
                                │
                    ┌───────────┼───────────┐
               ROS2 Path     JSON导出    PNG可视化
```

---

## 2. 算法选型

### 2.1 整体技术栈

| 组件 | 选用方案 | 对比方案 | 取舍理由 |
|------|----------|----------|----------|
| **覆盖路径库** | Fields2Cover 2.0 | 自研、OpenCover | 完整工具链（分解/条带/路径/TSP），社区活跃 |
| **区域分解** | Sweep 扫描线分解 | Boustrophedon 网格分解 | S3 cell 42→7，S6 加速 20-50x；网格产生过多梯形 cell |
| **条带排序** | Boustrophedon (来回扫描) | Snake (蛇形)、Spiral (螺旋) | Snake 实测穿洞反增（S3 6段 vs Boustrophedon 0段），已舍弃 |
| **掉头规划** | Direct Route 折线 | Dubins、Reeds-Shepp | 差速驱动可原地转；逐段保留 Route 绕行控制点 |
| **TSP 路由** | OR-Tools genRoute | 自研圆形绕孔洞排序 | genRoute 对 cell 内 swath 排序成熟可靠 |
| **路径简化** | RDP (Ramer-Douglas-Peucker) | — | 标准算法，减少路径点 60-80% |
| **覆盖评估** | 网格采样法 | 几何法 (F2C 原生) | 网格法更准确，不受 headland 干扰 |
| **边界处理** | Headland + fillBoundaryGaps + v7 段级检测 | 纯 headland 绕行 | 三阶段保障：外环补刀 + 孔洞补刀 + 段级覆盖检测回填 |

### 2.2 Snake 模式为什么被舍弃

Snake 模式假设相邻 cell 之间直接连接即可实现无穿洞连续扫描。

**实测结果**：
- S3 (含孔洞)：Snake 简化路径 6 段穿洞 > Boustrophedon 0 段
- Snake 覆盖率含孔洞穿越贡献（路径穿过孔洞 → 评估虚高）
- 真实覆盖并无优势，反而增加路径穿越孔洞风险

**决策**：Snake 从生产路线图移除，仅保留 Boustrophedon。

### 2.3 Sweep vs 网格分解

| 维度 | Sweep 分解 (当前) | Boustrophedon 网格 (旧) |
|------|-------------------|------------------------|
| 分解原理 | 孔洞顶点 y 坐标做水平切分线 | 梯形分解 + 合并 |
| S3 cell 数 | 7 (合并后 6) | 42 |
| 边界对齐 | 全宽条带，边界精确 | 梯形斜边，边界锯齿 |
| 孔洞处理 | 自然排除（孔洞顶点就是切分线） | 需额外 boundary fill |

### 2.4 Direct vs Dubins 掉头

| 维度 | Direct (当前) | Dubins |
|------|:---:|:---:|
| 连接方式 | Route 控制点间逐段直连 | 最小转弯半径曲线 |
| 适用场景 | 差速驱动（可原地转） | 阿克曼转向 |
| 路径点数 | 少 | 多（曲线离散化） |
| 计算速度 | 快 | 慢 |

若实车差速底盘可近似原地转，Direct 是首选；最终仍需结合底盘执行能力确认。

---

## 3. 核心逻辑

### 3.1 主流程

```
                    输入: Polygon (外环+孔洞)
                              │
                    ┌─────────▼─────────┐
                    │  1. 预处理         │
                    │  - 小孔洞过滤       │
                    │  - 自适应 Headland  │
                    │  - 窄通道检测       │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  2. 区域分解       │
                    │  rectilinearDecompose │
                    │  → 全宽条带 Cells   │
                    │  → simplifyRing     │
                    │  → filterTinyCells  │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  3. Swath 生成     │
                    │  - extractEdgeAngles│
                    │  - optimizeSwathAngle│
                    │  - adjustSwathEndpoints│
                    │  - fillBoundaryGaps │ ← 含 getLinesInside 裁剪
                    │  - Cell连接优化    │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  4. 路径规划       │
                    │  - TSP genRoute    │
                    │  - Direct 掉头     │
                    │  - simplifyPathRDP │
                    │  - v7 段级补线     │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  5. 评估 & 输出    │
                    │  - computeCoverageGrid│
                    │  - CoverageGate 评分│
                    │  - JSON/PNG 导出   │
                    │  - ROS2 Topic 发布 │
                    └───────────────────┘
```

### 3.2 关键函数

| 函数 | 所在模块 | 行数 | 功能 |
|------|----------|:--:|------|
| `planCoveragePath()` | polygon_planner_node | ~2000 | 主规划入口：协调分解→swath→路径→评估全流程 |
| `rectilinearDecompose()` | decomposer | ~160 | Sweep 扫描线分解：孔洞顶点 y 坐标 → 水平切分 → 全宽条带 |
| `fillBoundaryGaps()` | boundary_filler | ~270 | 边界补刀（完整版）：外环平行边向内偏移 → getLinesInside 裁剪 → 孔洞补刀 |
| `optimizeSwathAngle()` | swath_generator | ~40 | 多角度尝试，选 swaths 数量最少的方案 |
| `detectSlantedBoundaryAngle()` | swath_generator | ~70 | 检测 cell 是否贴近斜边（15°~75°），若是则返回斜边方向 |
| `simplifyPathRDP()` | path_planner | ~100 | RDP 路径简化：转弯感知分段 + 段内 RDP |
| `computeCoverageGrid()` | coverage_evaluator | ~100 | 网格覆盖评估：路径点 → 距离场 → 覆盖统计 |
| `pointInPolygon()` | boundary_filler | ~20 | 射线法点-in-多边形检测（Even-Odd Rule） |
| `segmentCrossesHole()` | boundary_filler | ~15 | 线段采样穿洞检测（用于 cell 连接优化） |

### 3.3 数据流

```
Polygon (YAML/ROS2) → polygon_planner_node
                           │
              F2C::Cell (外环+孔洞)
                    │
              F2C::Cells (Sweep分解后, ~7 cells)
                    │
              ┌─────▼─────┐
              │ Per Cell:  │
              │  - 斜边检测 │
              │  - 多角度优化│
              │  - 端点调整 │
              │  - 边界补刀 │
              │  - 孔洞补刀 │
              └─────┬─────┘
                    │
              F2C::SwathsByCells
                    │
              TSP genRoute → F2C::Route
                    │
              Direct Route 折线 → F2C::Path（控制点保真）
                    │
              ┌───────┴───────┐
              │               │
      非direct才做RDP       v7 段级检测
                         缺失swath回填
              │               │
              └───────┬───────┘
                      │
              完整路径 (用于评估)
              简化路径 (用于发布/可视化)
                      │
              ┌───────┼───────────┐
              │       │           │
         评估结果   JSON导出    ROS2 Path
         (覆盖率)   (vis+grid)  (RViz)
```

### 3.4 fillBoundaryGaps 边界补刀机制

**问题**：swath 排完后，最外层到 cell 边界的剩余距离不够一条 swath。不补这些间隙会导致边界区域覆盖缺失。

**算法**（v9 完整版，位于 `boundary_filler.cpp`）：

1. **外环补刀**：遍历多边形外环的所有边
   - 检测与 swath 方向平行（夹角 < 20°, cos > 0.9397）的边
   - 计算法向（指向 cell 质心 = 内侧）
   - 边向内偏移 `half_w = cov_width / 2`
   - `getLinesInside(full_polygon)` 裁剪（确保不超出多边形、不穿过孔洞）
   - 判断在出发侧还是到达侧 → 插入正确位置

2. **孔洞补刀**：遍历所有孔洞环
   - 同理检测平行边 + bbox 贴近
   - 法向指向外侧（远离孔洞中心）
   - 偏移 → getLinesInside 裁剪 → 插入

3. **v7 段级检测**：路径生成后追加检查
   - 逐段检测相邻路径点是否沿 swath 方向对齐
   - 找到最长连续对齐段序列
   - 跨度 < 50% swath 长度 → 判定为未覆盖 → 追加补线路径点

**Phase 2B 放宽容差**：角度容差从 cos(5°) = 0.996 放宽到 cos(20°) = 0.9397，覆盖更多斜边缝隙。

### 3.5 Cell 连接优化

**问题**：多 cell 场景中，cell 间连接不考虑起止点位置，产生不必要的长距离斜线。

**修复**：Boustrophedon 排序后，对每个 cell 测试反转 swath 顺序是否缩短连接距离。反转至少缩短 30% 且不穿越孔洞时才执行。

### 3.6 Per-Cell 离群 Veto（方向 B）

**问题**：全局角度优化为所有 cell 选统一 swath 角度（最少总 swath 数）。但某些 cell 在全局角度下 swath 数显著多于其本地最优角度——"离群 cell"。

**算法**（位于 `swath_generator.cpp:351-404`）：

```
全局角度优化 → 选定 best_ang（全 cell 最少 swath 总数）
  ↓
逐 cell 检查：
  1. 计算 cell 本地最优角（computeCellMainDirection + detectSlantedBoundaryAngle）
  2. 在本地候选角度（边缘角度 + 全局候选 + 主方向）中选最少 swath 数 → local_best
  3. 角度偏差 guard：本地角偏离全局 > 30° 时跳过（防止 cell 间角度不一致）
  4. 离群判断：全局 swath 数 > local_best × 2.0 且多 ≥ 3 条
     → 是：应用本地角度 + fillBoundaryGaps + filterShortSwaths
```

**设计考量**：
- 2.0x + 3 阈值经批测验证不触发非离群 cell，避免不必要的角度变更
- 30° 偏差 guard 确保 cell 间 swath 方向基本一致，减少连接段混乱
- veto 后仍做 fillBoundaryGaps 确保边界覆盖

### 3.7 孔洞穿越防护

**问题**：per-cell veto 导致 cell 间 swath 角度不一致时，greedyCellOrder DP 可能选择穿越孔洞的 cell 连接路径。

**方案**（多层防御）：
1. **硬约束** (`path_planner.cpp`)：`connectionCost` 对穿洞连接 `+= 1e9`，DP 视穿洞为不可行
2. **绕行修复** (`repairRouteConnectionsAroundHoles`)：沿孔洞边界生成绕行 polyline
3. **最终检测** (`countCrossings`)：路径仍穿洞则标记 plan failed

**v9.11 修复**：穿洞惩罚从 `+1000`（软约束）改为 `+1e9`（硬约束），根除 S7 穿洞问题。

### 3.8 自适应 Headland

检测多边形最窄通道宽度。若 headland 侵蚀会堵死通道（剩余宽度 < 有效覆盖间距），自动收窄侵蚀量，确保窄通道不被掐断。

---

## 4. 参数说明

### 4.1 机器人参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 车体宽度 | `robot_width` | 1.0 | 0.3~2.0 m | **0.75** | 源码默认值为占位值；推荐值来自实车测量 |
| 覆盖宽度 | `coverage_width` | 1.0 | 0.2~1.2 m | **约 0.90** | 两清洁轮组合名义宽度；冻结前需确认连续有效清洁带 |
| 最小转弯半径 | `min_turning_radius` | 0.01 | 0~1.0 m | **待确认** | 原 demo 使用 0.10 m；差速驱动是否可近似原地转需结合底盘确认 |

### 4.2 分解参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 启用分解 | `decomposition_enabled` | true | bool | **true** | false=全场一条条带 |
| Sweep 分解 | `use_sweep_decomp` | true | bool | **true** | 孔洞顶点切分，大幅减少 cell 数 |
| Cell 合并阈值 | `merge_angle_threshold` | 60.0 | 30~90° | **60°** | 相邻 cell 合并的角度差异上限 |
| 过滤微小 cell | `filter_tiny_cells` | true | bool | **true** | 面积 < 2×cov×robot 的 cell 忽略 |

### 4.3 Swath 参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 角度优化 | `swath_angle_optimization` | true | bool | **true** | 多角度尝试选最优 |
| 重叠率 | `swath_overlap_ratio` | 0.03 | 0~0.2 | **0.03** | 3% 重叠防漏缝 |
| 最小长度 | `min_swath_length` | 0.5 | 0.2~2.0 m | **0.5** | 短于此丢弃 |
| 端点收缩 | `swath_endpoint_shrink_distance` | 0.03 | 0~0.5 m | **0.03** | 闭合边界留转弯空间 |
| 排序算法 | `swath_order_type` | boustrophedon | boustrophedon | **boustrophedon** | 仅此选项，snake 已废弃 |

### 4.4 Headland 参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| mid_hl 比例 | `mid_hl_width_ratio` | 0.20 | 0.1~0.3 | **0.20** | 第一圈 headland 宽度 |
| no_hl 比例 | `no_hl_width_ratio` | 0.0 | 0~0.5 | **0.0** | 0=不生成第二圈 |
| 最小孔洞面积 | `min_hole_area` | 1.0 | 0.1~10.0 m² | **1.0** | 小于此忽略 |

### 4.5 路径规划参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 优化模式 | `use_optimized_planner` | true | bool | **true** | 总开关：启用所有优化 |
| 掉头类型 | `turn_planner_type` | direct | direct | **direct** | Direct=Euclidean 直连 |
| RDP 简化 | `path_simplify_enabled` | true | bool | **true** | 减少非 direct 路径冗余点；direct 为保护绕行控制点会跳过 |
| RDP 容差 | `path_simplify_tolerance` | 0.05 | 0.01~0.5 m | **0.05** | 越大越激进 |
| 路径分辨率 | `path_resolution` | 0.1 | 0.05~0.5 m | **0.1** | 路径点间距 |

### 4.6 边界策略参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 边界类型 | `boundary_type` | closed | closed/open/custom | **closed** | closed=物理墙(内缩)，open=虚拟边界(外伸) |
| 覆盖边距 | `boundary_coverage_margin` | -0.3 | -1.0~1.0 m | **-0.3** | 负=延伸，正=收缩 |
| 开放边距 | `boundary_open_default_margin` | -0.3 | -1.0~0 m | **-0.3** | open 模式默认延伸量 |

### 4.7 评估参数

| 参数 | ROS Key | 默认值 | 范围 | 推荐值 | 说明 |
|------|---------|--------|------|:------:|------|
| 启用评估 | `eval_enable_report` | true | bool | **true** | 生成覆盖率报告和 JSON |
| 网格分辨率 | `eval_grid_resolution` | 0.1 | 0.05~0.5 m | **0.1** | 越小越精确但越慢 |
| 网格方法 | `eval_use_grid_method` | true | bool | **true** | 比几何法更准确 |
| 覆盖阈值 | `eval_coverage_threshold` | 0.995 | 0.9~1.0 | **0.995** | 目标覆盖率，触发 RemArea 填补 |
| 最大曲率 | `max_diff_curv` | 0.3 | 0.1~0.5 | **0.3** | 掉头可行性判断 |

---

## 5. 评估体系

### 5.1 综合评分公式 (v8)

**设计理由**：实际环境下 100% 覆盖率几乎不可能达成。旧公式 `gate = coverage^10` 在 95% 时仍给 0.60 分，无法区分优劣。v8 将评分锚定在 99%（优秀）和 90%（不合格）。

**CoverageGate（覆盖率门控）**：三次幂陡降函数。

```
x = (coverage - 0.90) / 0.09
gate = x³    // 钳制到 [0, 1]
```

| 覆盖率 | gate | 说明 |
|:------:|:----:|------|
| ≥99% | 1.00 | 满分段 |
| 97% | 0.47 | 中等 |
| 95% | 0.17 | 偏差较大 |
| 93% | 0.04 | 接近底线 |
| ≤90% | 0.00 | 不合格 |

**综合得分 = gate × efficiency_score**

efficiency_score 综合：
- 工作比 (work_ratio)：理论最小长度 / 实际路径长度
- 重叠率 (overlap)：exp(-overlap_rate)
- 转弯数 (turn)：归一化
- 路径平滑度 (smoothness)：曲率分布
- 掉头可行性 (feasibility)：不可执行掉头比例

> **注意**：CoverageGate 是乘性门控，不是加法加权。覆盖率 < 90% 时整个得分归零。

### 5.2 最新基准 (v9.11, batch_0720_0245)

| 场景 | 面积 | 覆盖率 | 得分 | gate | hole_crossings | 说明 |
|------|:----:|:-----:|:----:|:----:|:---:|------|
| S1 矩形 20×15m | 300 | **100.00%** | **96.6** | 1.00 | 0 | 满分段 |
| S2 L 形 | 397 | **100.00%** | **94.9** | 1.00 | 0 | 满分段 |
| S3 含孔洞 | 458 | **100.00%** | **75.1** | 1.00 | 0 | 满分段 |
| S4 窄走廊 | 40 | **100.00%** | **89.1** | 1.00 | 0 | ✅ 近重合修复生效 |
| S5 不规则 | 90 | **100.00%** | **84.1** | 1.00 | 0 | 满分段 |
| S6 多区域 | 84 | **100.00%** | **72.3** | 1.00 | 0 | 满分段 |
| S7 工厂车间 | 600 | **99.81%** | **62.2** | 1.00 | **0** ✅ | 10孔洞场景 |
| S8 缺口+中心孔 | 297 | **100.00%** | **79.2** | 1.00 | 0 | 满分段 |
| **平均** | | **99.98%** | **81.7** | | | |

> 8/8 场景 hole_crossings = 0，v9.11 穿洞硬约束修复生效。S7 剩余 0.19% 已定位为物理不可达空隙（15 处，0.59m²）。

### 5.3 测试场景说明

| 场景 | 描述 | 面积 | 孔洞 | 挑战点 |
|------|------|:--:|:----:|--------|
| S1 | 矩形 20×15m | 300 m² | 0 | 边界覆盖完整性 |
| S2 | L 形 | 397 m² | 0 | 非凸形状分解 |
| S3 | 含孔洞 | 458 m² | 3 | 孔洞绕行、穿越避免 |
| S4 | 窄走廊 | 40 m² | 0 | 窄通道 (<2m)、转弯受限 |
| S5 | 不规则 | 90 m² | 0 | 复杂轮廓 |
| S6 | 多区域 | 84 m² | 2 | 多区域连接 |
| S7 | 工厂车间 30×20m | 600 m² | 10 | 多孔洞 + 复杂障碍物布局 |
| S8 | 缺口 + 中心孔 | 297 m² | 1 | 缺口边界 + 中心孔洞 |

---

## 6. 已知问题与技术债

### 6.1 已知问题

| 问题 | 严重度 | 状态 | 说明 |
|------|:------:|:----:|------|
| S4 窄走廊 ~90% | ⚠️ 中 | ✅ 已修复 (v9.7) | 近重合 swath 边界锚定等距重排，S4 100%/89.1 |
| A* 自由空间路由 | ⚠️ 中 | 🚧 开发中 | v9.9 启动，替代 boundary-walk 修复 cell 间连接；VM 崩溃待排查 |
| `/tmp/` 硬编码路径 | 🔵 低 | ✅ 已修复 (v9.7) | `artifact_sink` 模块使用操作系统临时目录 |
| 部分模块函数未被节点调用 | 🔵 低 | 渐进迁移 | filterShortSwaths、optimizeSwathAngle 等保留在节点内联实现中 |

### 6.2 技术债

| 项目 | 优先级 | 预估 | 说明 |
|------|:------:|:----:|------|
| 节点中剩余内联函数迁移到模块 | 🟡 中 | 1-2天 | filterShortSwaths、optimizeSwathAngle、detectSlantedBoundaryAngle 等尚未委托 |
| 参数配置外置到 YAML | 🟡 中 | 1天 | 30+ 参数散落命令行，应集中到 config YAML |
| 单元测试框架 | 🟡 中 | 2-3天 | 缺少单元测试，依赖手动基准 |
| VM LD_LIBRARY_PATH 自动化 | 🟢 低 | 0.5天 | 写入 ~/.bashrc 或 setup.bash |
| 孔洞穿越检测性能 | 🟢 低 | 0.5天 | O(n×m×50) 线段检测可优化 |

### 6.3 废弃方案记录

| 方案 | 废弃日期 | 原因 |
|------|:--------:|------|
| Snake 路径模式 | 07-14 | S3 穿洞 6 段，覆盖率虚高 |
| Boustrophedon 网格分解 | 07-12 | Sweep 全面优于网格法 |
| 几何评估法 | 07-11 | 网格法更准确 |
| fillBoundaryGaps 去重 | 07-11 | 误删合法边界补刀 |
| 自适应 headland (两级) | 07-10 | no_hl=0 时无效果 |
| C-space 膨胀层 | 07-19 | 由空隙分类器替代，更精细区分物理不可达 vs 算法可改进 |
| Snake 路径模式 | 07-14 | S3 穿洞 6 段，覆盖率虚高 |

### 6.4 代码审查发现 (v9, 2026-07-14)

| 问题 | 级别 | 处置 |
|------|:--:|------|
| fillBoundaryGaps 模块/节点算法不同 | ERROR | ✅ 已修复：完整提取到 boundary_filler.cpp |
| repairConnection 退化路径 (1点) | ERROR | ✅ 已修复：始终保留双端点 |
| simplifyCells hole_angle_tol_deg 丢弃 | ERROR | ✅ 已修复：恢复独立孔洞容差 |
| simplifyPathRDP 参数不同 (2参 vs 3参) | INFO | 📋 有意为之：模块不依赖 ROS 成员变量 |
| 部分模块函数 "僵尸代码" | INFO | 📋 渐进迁移策略 |

---

## 7. 运行方式

### 7.1 系统依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| Ubuntu | 22.04 LTS | 操作系统 (VMware 虚拟机) |
| ROS2 | Humble | 消息通信、参数管理 |
| Fields2Cover | 2.0+ (工作空间内编译) | 全覆盖路径规划核心库 |
| OR-Tools | 9.x | TSP 路由优化 |
| GDAL | 3.x | 多边形几何运算 |
| Python 3 | 3.10+ | 可视化渲染脚本 |
| PyYAML + Matplotlib | — | YAML 解析 + PNG 渲染 |

### 7.2 VM 环境编译

```bash
# 1. 环境
source /opt/ros/humble/setup.bash

# 2. 编译 (CMakeLists 已配置 Fields2Cover build 路径)
cd ~/f2c_coverage_planner
colcon build --packages-select yingshi_robot --symlink-install

# 3. 环境变量 (建议加到 ~/.bashrc)
source install/setup.bash
export LD_LIBRARY_PATH=$HOME/f2c_coverage_planner/install/lib:\
$HOME/f2c_coverage_planner/src/Fields2Cover/build/_deps/steering_functions-build:\
$HOME/f2c_coverage_planner/src/Fields2Cover/third_party/ortools-src/lib:\
${LD_LIBRARY_PATH}
```

### 7.3 RViz 可视化（VM 端）

```bash
export LIBGL_ALWAYS_SOFTWARE=1  # VMware 黑块修复
bash ~/f2c_coverage_planner/install/yingshi_robot/share/yingshi_robot/scripts/run_f2c_optimized.sh
```

### 7.4 Windows 端一键操作

```bash
# 在 Git Bash 中，cd 到 f2c_coverage_planner/
bash sync_and_build.sh              # 仅同步+编译
bash sync_and_build.sh --test S1    # 单场景测试
bash sync_and_build.sh --batch      # 7场景全量基准+可视化报告
bash sync_and_build.sh --run        # VM端启动RViz
```

`--batch` 自动完成：同步源码 → VM编译 → 7场景测试 → PNG渲染 → 拉回结果。

### 7.5 配置文件

多边形场景文件（YAML 格式）：

```
src/yingshi_robot/test_polygons/   — S1~S7 测试多边形
src/yingshi_robot/config/f2c_areas/ — S8 等实际场景
```

**YAML 格式**：
```yaml
polygon:
  - [x1, y1]
  - [x2, y2]
  - ...
holes:
  - - [hx1, hy1]
    - [hx2, hy2]
    - ...
boundary_type: closed
```

### 7.6 输出文件

| 文件 | 路径 | 说明 |
|------|------|------|
| Vis JSON | `/tmp/f2c_vis_polygon_1.json` | 路径点 + 评估结果 |
| Grid JSON | `/tmp/f2c_grid_polygon_1.json` | 网格覆盖数据 |
| 渲染 PNG | `test_results/batch_*/` | 路径图 + 覆盖率热力图 |
| ROS2 Topic | `/planned2_path_1` | 实时路径消息 (RViz) |

### 7.7 手动渲染

```bash
python3 scripts/render_coverage.py <场景名> <yaml> <data.json> <output.png>
```

### 7.8 从 JSON 快速查看结果

```bash
python3 -c "
import json
with open('/tmp/f2c_vis_polygon_1.json') as f:
    d = json.load(f)
e = d['eval']
print(f'覆盖率: {e[\"coverage_rate\"]*100:.1f}%')
print(f'得分: {e[\"single_score\"]:.1f}')
print(f'路径点: {len(d[\"path\"])}')"
```

---

> **最后更新**: 2026-07-20 (v9.11 穿洞硬约束修复 + per-cell 离群 veto + S8 重命名)
> **相关文档**: `SETUP_GUIDE.md` (发布包配置指南)、`daily_logs/2026-07-20.md`、`产品需求待办.md`
