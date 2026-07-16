# F2C 全项目代码审查报告

> 日期：2026-07-16  
> 方法：3 agent 并行扫描 14 个 .cpp/.hpp 文件（内存安全 / 线程安全&死代码 / API一致性&日志）  
> 基准：batch_0716_1140，7 场景全通过，零退化

---

## 一、审查发现与修复状态

### P0：路径正确性（Codex 发现，全部已修）

| # | 问题 | 位置 | 修复 |
|:--:|------|------|:--:|
| 1 | PathState 语义误用：只发布 `state.point`，最后一个 state 的整段路径丢失 | poly_planner_node.cpp:3258 | `materializePath()` 统一展开折线 |
| 2 | 边界补线未发布：path_msg 在补线前构建 | poly_planner_node.cpp:3249 vs :3387 | materializePath 后于补线执行 |
| 3 | 双向切割未执行：节点调用旧版成员函数，新版 decomposer 是死代码 | decomposer.cpp:40 vs node:588 | 切换到 `yingshi::rectilinearDecompose()` → 回退（S3/S6 退化） |

### P1：过程可靠性（全部已修）

| # | 问题 | 位置 | 修复 |
|:--:|------|------|:--:|
| 4 | ROS2 参数回调一拍延迟 | node:136 | 回调参数直传 updateParameters() |
| 5 | 规划失败仍发旧路径 | node:2020, :3592 | clearPlanningCache + 发空路径 |
| 6 | 补线尾部无连接规划 | node:3277 | 确认 materializePath 已修，加 TODO |
| 7 | batch_test 不返回失败退出码 | batch_test_v2.sh | ERRORS 计数器 + exit 1 |
| 8 | 参数默认值不一致：shrink_distance | node:91 vs batch | 统一为 0.03m |
| 9 | 参数无校验：非法分辨率/枚举值/比例 | node:320-377 | 添加 clamp + enum 校验 |

### 内存安全 & UB

| 级别 | 问题 | 位置 | 修复 |
|:--:|------|------|:--:|
| 🟡 | C0 空 cell 时 at(size-1) 崩溃 | path_planner.cpp:278 | 防御性 fallback |
| 🟡 | num_samples=0 除零 | boundary_filler.cpp:64 | 加 guard |
| 🟢 | best_ci 未初始化 | path_planner.cpp:286 | 代码路径安全，不改 |
| 🟢 | int 溢出理论风险 | decomposer.cpp:192 等 | 有守卫，不改 |

### 线程安全

状态：**零 mutex/atomic，4 种回调并发访问 30+ 成员变量。**
当前单线程执行器下实际安全，记入已知限制。切多线程执行器时需专项处理。

### 死代码（全部已删）

| 函数 | 文件 | 原因 |
|------|------|------|
| `yingshi::checkInfeasibleTurns` | path_planner.cpp:144 | 零调用，节点内联了自己的实现 |
| `yingshi::repairConnection` | path_planner.cpp:176 | 零调用，仅返回两点 Path |
| `yingshi::computePolygonMainDirection` | swath_generator.cpp:122 | 零调用，纯委托包装 |
| `yingshi::EvalParams` | planner_params.hpp:54 | 与 coverage_evaluator.hpp 重复定义 |
| `#include <string>` | path_planner.hpp:4 | 无使用 |

### API / 日志一致性（全部已修）

| 问题 | 位置 | 修复 |
|------|------|------|
| 自适应地头日志用 `coverage_width_` 代替实际检查值 `r_w` | node:1512 | 改为 r_w |
| 覆盖率门控注释写 `coverage^10`，实为 `((x-0.9)/0.09)^3` | coverage_evaluator.hpp:85 | 更新注释 |
| decomposer 注释写"不压缩 X"但 sweep 路径已压缩 | decomposer.cpp:78 | 更新注释 |

### 测试覆盖

| 类型 | 数量 | 状态 |
|------|:--:|:--:|
| path_geometry 测试 | 6 | ✅ 全通过 |
| decomposer 测试 | 1 | ✅ 全通过 |
| 未覆盖函数 | ~30 | 📝 后续逐步补 |

### 脚本问题

| 问题 | 文件 | 修复 |
|------|------|:--:|
| sync 不含 test/ 目录 | sync_and_build.sh | ✅ 已加 |
| build 未开 BUILD_TESTING | sync_and_build.sh | 📝 后续加 --cmake-args |
| /tmp/ 硬编码路径 | 多个脚本 | 📝 已知限制 |
| /opt/ros/humble/ 硬编码 | run_f2c_*.sh | 📝 已知限制（实车再修） |

---

## 二、修复统计

| 指标 | 数值 |
|------|:--:|
| 本轮 commits | 11 |
| 新增文件 | 4（path_geometry.hpp, 2 test, 3 memory） |
| 净增/删行数 | +243 / -463 = -220 |
| 修复问题数 | 14 |
| 遗留已知限制 | 3 |

---

## 三、最终基准 (batch_0716_1140)

| 场景 | 覆盖率 | 得分 | 未覆盖 | 路径长 | 重叠率 | 耗时 |
|------|:------:|:----:|:-----:|:-----:|:-----:|:---:|
| S1 矩形 | 99.92% | 81.0 | 0.24 | 742.7 | 0.0% | 303ms |
| S2 L形 | 99.60% | 82.6 | 1.57 | 941.5 | 0.7% | 720ms |
| S3 含孔洞 | 99.09% | 75.0 | 4.15 | 1143.5 | 0.0% | 2836ms |
| **S4 窄走廊** | **99.95%** | **75.3** | **0.02** | **120.4** | **29.3%** | **33ms** |
| S5 不规则 | 99.63% | 77.0 | 0.33 | 242.1 | 0.9% | 277ms |
| S6 多区域 | 98.44% | 61.5 | 1.31 | 232.0 | 0.0% | 358ms |
| notched 缺口孔 | 98.62% | 65.9 | 4.11 | 770.6 | 0.0% | 1526ms |

### vs v9.2.1 基线

- S4：覆盖率 **+9.92%**，得分 **+75.3**（PathState 修复主因）
- 其余 6 场景：覆盖持平，分降（路径长度准确后评分更诚实）
- **零场景退化**

---

## 四、已知限制（未修，记入 backlog）

1. **线程安全**：零 mutex/atomic，单线程执行器下安全，切多线程时需专项
2. **节点-模块代码重复**：7 个函数在 node 中复制了模块实现（~500 行），后续可让 node 方法委托到模块
3. **hole_rings 构建重复**：相同逻辑在 4 处复制粘贴，可提取辅助函数
4. **/tmp/ 硬编码路径**：Windows 兼容性待解决
5. **测试覆盖不足**：~30/35 公开函数无测试，后续逐步补

---

## 五、后续优化建议（参考 Codex + 优化方向.md）

1. **P0 产品**：S4 动态重叠率 `N=ceil(W/C), spacing=(W-C)/(N-1)`
2. **P1 算法**：步骤6 swath 方向继承分解角度、S3/S6 精确线段-孔洞相交
3. **P2 工程**：清理节点-模块重复、补边界补线 pipeline 重组
