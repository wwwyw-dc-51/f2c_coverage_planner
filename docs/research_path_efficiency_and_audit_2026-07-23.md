# 路径效率与审计脚本误报研究

日期：2026-07-23

范围：只读检查当前代码、批测 JSON 和旧日志；本轮未修改规划代码，也未运行 VM 测试。

## 结论摘要

1. “第三轮 V-merge”当前确实基本是重复收尾：`mergeAdjacentSweepStrips()` 在 `planner_core.cpp:385` 和 `planner_core.cpp:393` 连续调用，中间只有 `normalizeCells()`，没有会产生新相邻条带的拆分步骤。因此第二次通常是幂等重跑，不能解决 N3 的碎片化。
2. `mergeAdjacentSweepStrips()` 的 `coverage_width` 参数目前被显式忽略（`decomposer.cpp:650-652`），所以调整“2m 货架条带合并阈值”不会影响这条 V-merge；真正使用 `2 * coverage_width` 的是 `mergeCellsWithSimilarDirection()`（`decomposer.cpp:471`）。
3. N3 的 101% 重叠和 S7 的长路径，主因不是连接 JSON 里的重复端点。重复端点只产生零长度段；S7 的主要成本是 16 个 cell、100 条 Swath、99 条 cell 间连接，连接长度约 164.1m。
4. 安全方向是“同一自由空间带内的共线 Swath 合并”，并在合并后重新做孔洞/规划区安全检查；不建议直接扩大所有 cell 的顶点邻近阈值，也不建议恢复全局 X 切割。
5. 审计脚本的转弯误报已经可以精确复现：评估器使用动态合并距离 `max(0.75, 1.05 * 0.90 * (1 - 0.03)) = 0.91665m`，审计器默认使用 0.75m。最新批次 S1/S2/S3/S5/S6/S7 的报告转弯数与审计器在 0.91665m 下完全一致。
6. 0.375m 净空误报来自绝对容差过小：S7 有路径点的实际计算净空为 `0.3749999711989731m`，距理论值 0.375m 只差约 `2.9e-8m`，当前只加 `1e-9` 仍会判违规。
7. 扩展场景 YAML 不是按报告名直接命名。批测脚本中的 14 个 `N*` 别名和 `S8` 都需要显式映射；当前审计器因此对 N1~N13、S8 报 `POLYGON_FILE_MISSING`。

## 一、N3/S7 路径效率

### 1. 当前数据

来自 `test_results/batch_0723_0326`：

| 场景 | Cell | Swath | 连接 | 转弯 | 路径总长 | Swath 工作长度 | 连接长度估算 | 重叠率 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| N3_dense | 21 | 88 | 87 | 107 | 546.141m | 427.628m | 118.513m | 101.0% |
| S7 | 16 | 100 | 99 | 128 | 896.864m | 732.738m | 164.126m | 53.8% |

连接长度按 `total_distance * (1 - work_ratio)` 估算。它说明 S7 的问题更接近“cell 间过渡多、每次过渡偏长”，而不是单纯的工作线重复。

N3 的 21 个 cell 来自密集货架孔洞分解。其孔洞间距只有 1.5m，经过 C-space/headland 处理后形成许多相互分离的窄条带。把这些条带强行合成一个含孔洞 cell，会改变拓扑，甚至为后续 Swath 生成和连接制造穿孔风险。

### 2. 当前合并链路的实际行为

`planner_core.cpp:373-380` 先执行 `mergeCellsWithSimilarDirection()`。它的候选条件是：

- cell 外环顶点最小距离小于 `2 * coverage_width`；
- 主方向在角度阈值内；
- 质心连线不能穿孔洞；
- union 后不能留下 interior ring。

因此“把 1.8m 阈值改大”只会放宽第一道门，不能绕过孔洞保护和 union 拓扑检查；如果放宽过度，反而可能把不应连通的区域送入后续规划。

随后 `planner_core.cpp:383-395` 连续执行两次 `mergeAdjacentSweepStrips()`。该函数只按以下固定条件合并：

- X 方向包围盒两端差值不超过 0.05m；
- 垂直间隙不超过 0.01m；
- union 后不产生 interior ring。

其签名中的 `coverage_width` 在 `decomposer.cpp:650-652` 被写成未命名参数，函数体没有使用它。第二次调用前只有 `normalizeCells()`，没有新的条带拆分，所以“第三轮 V-merge”当前没有独立算法效果，最多是拓扑归一化后的重复检查。

### 3. 适合试验的方向

优先研究 Swath 层，而不是扩大 cell 层合并：

1. 在每个 cell 内生成 Swath 后，按主方向归一化，识别同一条自由空间带中的共线片段。
2. 只合并满足以下条件的片段：方向差在小角度内、投影区间相邻或重叠、横向偏差小于容差、连接线段不穿孔洞且不离开 planning cell。
3. 合并后重新计算 Swath 端点和 route connection，再统计覆盖与路径长度。
4. 对 N3 重点看是否减少重复扫掠；对 S7 重点看是否减少 cell 交界处的独立过渡次数。

候选实验应按低风险顺序进行：

- 先做仅删除连续重复端点的连接输出清理；
- 再做“同方向、同自由空间带、无障碍穿越”的 Swath 合并；
- 最后才评估 cell 合并阈值，且必须逐场景比较 cell 数、hole crossing、覆盖率和连接长度。

### 4. 连接路径去冗余的边界

最新 JSON 中，N3 连接路径连续重复点约 156 个，S7 约 195 个。原因是输出层把 `from_point`、Route connection 内点、`to_point` 依次写出，而 Route connection 自身常已包含端点。

这类重复点不会增加几何长度：`path_planner.cpp` 的 `appendDistinct()`、`appendSegment()` 和 `path_geometry.hpp` 的 `materializePath()` 已经在实际路径展开时跳过连续重复点。因此：

- JSON 序列化层去重是低风险、可做的清理；
- 它能减少点数和审计噪声，但不会降低 N3 的 101% 重叠率，也不会实质缩短 S7 的 896.9m；
- 真正需要优化的是连接控制点产生的折线路径和 cell 顺序，而不是零长度重复端点。

## 二、`audit_test_report.py` 误报

### 1. 转弯统计不是同一口径

审计器在 `scripts/audit_test_report.py:29-48` 对最终路径折线的几何方向变化计数；评估器在 `coverage_evaluator.hpp:392-425` 也使用类似几何算法，但评估调用传入的合并距离来自 `coverage_evaluator.hpp:725-732`：

```text
effective_width = coverage_width * (1 - swath_overlap_ratio)
merge_distance = max(configured_minimum, 1.05 * effective_width)
```

当前基线为 `coverage_width=0.90`、`overlap=3%`、最小合并距离 `0.75m`，所以实际值是 `0.91665m`，不是审计器 CLI 默认值 `0.75m`。

复核结果：

| 场景 | 报告值 | 审计器 0.75m | 审计器 0.9165m |
|---|---:|---:|---:|
| S1 | 17 | 32 | 17 |
| S2 | 32 | 58 | 32 |
| S3 | 70 | 95 | 70 |
| S5 | 25 | 33 | 25 |
| S6 | 27 | 36 | 27 |
| S7 | 128 | 149 | 128 |

所以当前 `TURN_COUNT_MISMATCH` 主要是参数契约缺失，不是规划路径真的错。更稳妥的修复是让 JSON 报告携带评估实际使用的 `turn_angle_threshold`、`turn_merge_distance`，审计器优先读取报告值；报告缺少这些字段时不应把几何复算差异升级为 error，最多给出“无法严格复核”的 warning。

### 2. 0.375m 边界浮点误报

审计器在 `scripts/audit_test_report.py:190-198` 以 `robot_width / 2` 作为净空要求，并用：

```python
boundary_clearance + 1e-9 < required_clearance
```

判断违规。S7 中存在路径点：

```text
理论净空：0.375m
实际计算：0.3749999711989731m
误差：约 2.88e-8m
```

这属于几何计算和十进制坐标传播造成的舍入误差，不应被视为真实碰撞。建议把净空判定改为统一的尺度相关容差，例如 `1e-6m` 起步，并把“明显低于要求”的真实缺口与“落在容差带内”的边界点分开统计。不要只在 `point_in_ring()` 上加容差，因为这里的误报发生在 `_boundary_clearance()` 的比较。

### 3. 扩展场景映射缺失

`audit_test_report.py:282-293` 只尝试按报告名拼接 YAML 文件名。批测脚本 `batch_test_v2.sh:18-39` 使用了显式别名，例如：

```text
S8        -> config/f2c_areas/notched_10m_with_center_hole.yaml
N1_ring   -> test_polygons/ring.yaml
N3_dense  -> test_polygons/dense_shelves.yaml
N10_whoriz -> test_polygons/warehouse_horiz_shelves.yaml
N13_robs  -> test_polygons/rect_multi_obstacles.yaml
```

当前运行审计器时，S8 和 N1~N13 全部产生 `POLYGON_FILE_MISSING`。建议把批测脚本的场景表抽成审计器可复用的显式映射，或让 JSON 保存 `scenario_yaml` / `polygon_file` 相对路径并以报告字段为准。仅继续扩大 glob 模式会把“找得到文件”和“找对文件”混在一起，存在误配风险。

## 建议的后续修复顺序

1. 先修审计输入契约：报告写入转弯阈值、动态合并距离和场景 YAML 相对路径。
2. 再修审计数值容差：净空比较使用统一的 `1e-6m` 量级容差，并增加恰好 0.375m、略低于 0.375m、明显低于 0.375m 三类回归样例。
3. 连接 JSON 做连续重复点去重，验证只影响序列化点数，不改变实际路径长度和安全检查。
4. 研究同自由空间带的共线 Swath 合并，分别观察 N3 的重叠率和 S7 的连接长度；合并必须通过孔洞穿越与 planning cell 安全门。
5. 暂不直接扩大 `2 * coverage_width`，也不把第二次 `mergeAdjacentSweepStrips()` 当作有效优化；若要保留第二轮，应让它处理真实的新输入或删除重复调用。
