# batch_0716_1331 路径诊断与参数校准

> 日期：2026-07-16
> 面向读者：继续修复本项目的开发者或 agent
> 证据目录：`test_results/batch_0716_1331/`
> 状态：诊断结论已确认；候选物理参数尚未写入运行脚本、尚未重跑基准

## 1. 先读结论

本轮需要区分两类问题：

1. **与产品参数无关的确定性代码缺陷**：Route 边界补线插入顺序、route connection 被拉直、孔洞诊断环未闭合。这三项应先修。
2. **物理模型参数错误**：历史批测使用 `robot_width=0.95`、`coverage_width=0.45`；实车测量得到车身约 `0.75 m`，两个对称软质清洁轮直径均为 `0.45 m`，名义组合清洁宽度约 `0.90 m`。

候选参数：

```text
robot_width=0.75
coverage_width=0.90        # 名义值；需用直线清扫痕迹确认连续有效宽度
swath_overlap_ratio=0.03   # 第一轮验证沿用 3%
```

在该候选模型下：车身半宽为 `0.375 m`，清洁半宽为 `0.45 m`。若清洁轮相对车身居中，路径中心距硬边界 `0.45 m` 时，理论车身净空约为 `0.075 m`，因此“覆盖墙根”和“不碰墙”可以同时成立。

## 2. 参数来源与可信度

### 2.1 实车测量（当前最高优先级）

- 车身宽度：约 `0.75 m`。
- 两个清洁轮：直径均为 `0.45 m`，对称安装，材质较软。
- 名义清洁宽度：约 `0.90 m`。
- 待补测：一次直线行驶留下的**连续有效清洁带宽度**，用于确认是否存在中间漏缝或边缘无效区。

### 2.2 2026-07-06 原始 demo

原始 demo 位于：

```text
C:\Users\19015\Documents\WeChat Files\wxid_fbe57g7lvfvp11\FileStorage\File\2026-07\f2c_demo_ws\f2c_demo_ws
```

其正式启动脚本 `src/yingshi_robot/scripts/run_f2c_rviz_demo.sh` 使用：

```text
robot_width=0.95
coverage_width=0.45
min_turning_radius=0.10
max_diff_curv=0.30
mid_hl_width_ratio=0.25
no_hl_width_ratio=0.00
swath_endpoint_shrink_distance=0.25
min_swath_length=0.50
```

这证明 `0.95/0.45` 来自原始 demo，但不能证明它们等于实车尺寸。实测值应覆盖 demo 假设。

### 2.3 历史报告限制

`batch_0716_1331` 仍使用 `0.95/0.45`。其中覆盖率、路径长度、swath 数量、碰撞净空和重叠率都不能直接作为新物理模型的验收基线。中心线穿出多边形属于纯几何错误，不受车宽参数变化影响，仍然有效。

## 3. 已确认的代码缺陷

### P0：边界补线破坏 Route 结构

位置：`src/yingshi_robot/src/polygon_planner_node.cpp` 的边界补线恢复逻辑。

当前顺序：

```cpp
route.addSwath(bsw);
route.addConnection(conn);
```

F2C `Route::addSwath()` 在 connection 数量未领先 swath group 数量时，会把新 swath 追加到最后一个已有 group；随后添加的 connection 则位于 route 尾部。因此补线前的连接没有位于补线前，路径规划会在组内直接连接两个远端 swath。

证据：

- S6 `seg[229]`：`(0.218,-0.110) -> (11.730,0.050)`，长 `11.51 m`，约 `7.9 m` 真正位于作业区外。
- S3 `seg[561]`：右侧约 `20.41 m` 的异常长连接，其中约 `2.2 m` 越界。
- notched 尾部存在约 `0.6 m` 越界。

修复原则：使用 `Route::addConnectedSwaths(connection, swaths)`，或严格保证“先创建属于新 group 的 connection，再添加新 group”。增加以下不变量测试：

- 每个 `connection[i]` 位于 `swaths[i]` 之前。
- 新增边界补线必须形成独立 swath group。
- 物化后的 S6 路径不存在上述 11.51 m 横跨段。

### P0：安全 route connection 被简化为端点弦线

位置：`src/Fields2Cover/src/fields2cover/path_planning/path_planning.cpp` 的 `PathPlanning::simplifyConnection()`。

证据：

- notched connection 79 原本含 30 个路点，沿 L 形区域底边和右边绕行，折线长 `18.06 m`。
- 最终路径 `seg[480]` 变成 `(0,0.218) -> (6,12.218)`，只有一条 `13.41 m` 斜线，首尾与原 connection 完全一致。
- S5 connection 14、23 同样由凹角折线退化为斜线。

根因：所谓 `direct` 模式仍调用 F2C `DubinsCurves` 和 `PathPlanning::simplifyConnection()`；简化器可能仅根据端点距离和平滑转弯半径丢弃中间控制点，没有验证弦线是否保持在可行区域内。

修复原则：

- direct 模式逐段保留 route waypoint 折线。
- 任何“折线变弦线”必须先验证弦线完全位于可行区域并满足碰撞净空。
- 单元测试必须覆盖带两个直角拐点的 L 形 connection。

### P1：孔洞交叉诊断使用未闭合环

位置：`polygon_planner_node.cpp` 中构造 `hole_check_rings` 的发布路径诊断逻辑。

诊断代码只加入孔洞顶点，没有补回“最后一点 -> 第一点”。射线法因此把孔洞左侧大面积区域误判为孔内。

使用闭合环重新计算：

| 场景 | 当前日志 | 正确结果 |
|---|---:|---:|
| S3 | 84 点、86 段穿孔 | 0 点、0 段 |
| S6 | 9 点、27 段穿孔 | 0 点、0 段 |
| notched | 137 点、137 段穿孔 | 0 点、0 段 |

不要继续根据这些“穿孔”日志设计绕洞算法。应先统一使用一个保证闭环的 hole-ring 构造函数，并增加矩形孔洞左右两侧的点包含测试。

## 4. 肉眼路径判断

| 场景 | 观察 | 判断 |
|---|---|---|
| S1 | 标准往复式路线 | 拓扑正常；旧参数下过密，新参数需重跑 |
| S2 | L 形区域路线基本自然 | 凹角仍需检查连接折线是否被拉直 |
| S3 | 主体 swath 合理，尾部出现异常长连接 | Route 边界补线插入错误；红色小缺口是真实零碎漏扫，不是穿孔 |
| S4 | 2 m 走廊内约 5 条密集路线 | 由旧 `coverage_width=0.45` 导致；新值预计降到约 2~3 条 |
| S5 | 两条跨凹角斜线 | route 折线被简化成端点直线；中心线基本仍在区域内 |
| S6 | 底部横跨外部，门洞上下存在硬切 | 当前最严重；先修 Route 插入顺序和连接简化 |
| notched | 左下到 `(6,12)` 的巨大斜线 | 正确 L 形绕行 connection 被拉成弦线 |

## 5. 新一轮验收顺序

1. 修复三个确定性代码缺陷，不先调评分权重。
2. 将测试脚本参数切换到 `robot_width=0.75`、`coverage_width=0.90`。
3. 重新生成 7 场景报告；旧批次只用于几何回归对照。
4. 检查硬门槛：
   - 正确闭环后孔洞交叉为 0。
   - S6、S3、notched 中心线越界长度小于 `0.05 m`。
   - 最小硬边界中心距至少为 `robot_width/2 + safety_margin`。
   - route 中的绕行控制点在最终 materialized path 中仍可识别。
5. 再评估覆盖率、路径长度、转弯数和重叠率。

建议首轮使用 `safety_margin=0.03 m`。在名义 `0.90 m` 清洁宽度下，路径中心距墙 `0.45 m`，对 `0.75 m` 车身仍保留约 `0.075 m`，扣除 3 cm 安全余量后尚有约 4.5 cm 容差。

## 6. 不要混淆的结论

- 旧报告中“按 0.95 m 车宽估算的大量净空失败”已被实车测量修正，不能继续当作产品碰撞结论。
- S3/S6/notched 的日志穿孔数字是未闭环诊断产生的误报。
- S6 的中心线真实越界、notched 的大斜线、Route 插入顺序错误仍是确定性问题。
- `coverage_width=0.90` 目前是名义候选值，最终应以连续清洁痕迹实测值为准。
