# F2C 全覆盖路径规划系统

> **版本**: v9.11 (2026-07-20) | **编译**: 0 error 0 warning | **测试**: 74/74 ✅
> 
> *v9.11 = v9.7 + C-space退役 + 空隙分类器 + Phase 4A + filter/merge顺序修复 + S8重命名 + per-cell离群veto + 穿洞硬约束修复*

基于 Fields2Cover + ROS2 Humble 的扫地机器人全覆盖路径规划系统。  
GitHub: [wwwyw-dc-51/f2c_coverage_planner](https://github.com/wwwyw-dc-51/f2c_coverage_planner)

> **物理参数校准（2026-07-16）**：实车测得车身宽约 `0.75 m`；两个直径 `0.45 m` 的软质清洁轮对称安装，名义清洁宽度约 `0.90 m`。历史批次仍使用原 demo 的 `0.95/0.45`，因此旧报告的路径长度、swath 数和碰撞净空不能作为新参数验收基线。详见 [`docs/diagnosis_batch_0716_1331.md`](docs/diagnosis_batch_0716_1331.md)。

---

## 快速开始

### 编译

```bash
cd f2c_coverage_planner
source /opt/ros/humble/setup.bash
colcon build --packages-select yingshi_robot --symlink-install
source install/setup.bash
```

### 运行单场景

```bash
# 终端1：启动 planner
ros2 run yingshi_robot polygon_planner_node --ros-args \
  -p robot_width:=0.75 -p coverage_width:=0.90 \
  -p use_sweep_decomp:=true -p swath_order_type:=boustrophedon \
  -p eval_enable_report:=true -p eval_use_grid_method:=true

# 终端2：发布多边形
python3 scripts/publish_polygon.py <yaml_path>
```

### 批量基准测试

```bash
python3 scripts/bench_7_v6.py
```

### 可视化

```bash
python3 scripts/render_coverage.py <label> <yaml> <vis_json> <output.png>
```

---

## 项目结构

```
f2c_coverage_planner/
├── src/yingshi_robot/
│   ├── src/
│   │   ├── polygon_planner_node.cpp   # 主规划器 (4180行)
│   │   ├── decomposer.cpp             # 区域分解模块 (340行)
│   │   ├── swath_generator.cpp        # Swath生成模块 (274行)
│   │   ├── boundary_filler.cpp        # 边界补刀模块 (276行)
│   │   ├── path_planner.cpp           # 路径规划模块 (202行)
│   │   └── coverage_evaluator.hpp     # 评估器 (~700行)
│   ├── include/yingshi_robot/         # 模块头文件 × 5
│   ├── test_polygons/                 # S1~S8 测试多边形
│   └── config/f2c_areas/             # 实际场景多边形 (S8)
├── scripts/
│   ├── bench_7_v6.py                  # 7场景批量基准
│   ├── render_coverage.py             # 路径可视化渲染
│   └── md2docx.py                     # Markdown→Word 转换
├── docs/
│   ├── F2C_技术文档.md/.docx           # 完整技术文档
│   ├── progress_report_2026-07-14_v8.md/.docx  # 综合进展报告
│   └── daily_logs/                    # 每日开发日志 (07-08~07-14)
├── test_results/
│   ├── v8_final/                      # v8 稳定版基准结果
│   └── archive/                       # 历史测试结果
├── releases/v8_2026-07-14/            # v8 发布包（源码+数据+可视化）
└── backups/                           # 源码备份
```

---

## 当前基准（v9.11，batch_0720_0245，实车参数）

> `robot_width=0.75`（实车）、`coverage_width=0.90`。所有场景 hole_crossings = 0。

| 场景 | 覆盖率 | 得分 | 路径长 | hole_crossings | 说明 |
|------|:------:|:----:|:-----:|:---:|------|
| S1 矩形 20×15m | **100.00%** | 96.6 | — | 0 | ✅ |
| S2 L 形 | **100.00%** | 94.9 | — | 0 | ✅ |
| S3 含孔洞 | **100.00%** | 75.1 | — | 0 | ✅ |
| S4 窄走廊 | **100.00%** | 89.1 | — | 0 | ✅ 近重合修复生效 |
| S5 不规则 | **100.00%** | 84.1 | — | 0 | ✅ |
| S6 多区域 | **100.00%** | 72.3 | — | 0 | ✅ |
| **S7 工厂车间 30×20m** | **99.81%** | 62.2 | — | **0** ✅ | 10孔洞，穿洞已修复 |
| S8 缺口+中心孔 | **100.00%** | 79.2 | — | 0 | ✅ |
| **平均** | **99.98%** | **81.7** | — | — | |

---

## 评分公式 (v8)

```
CoverageGate = ((coverage - 0.90) / 0.09)³   // 钳制到 [0, 1]
综合得分 = CoverageGate × efficiency_score

≥99% → gate=1.0 (满分)    90% → gate=0 (不合格)
```

> 理由：实际运行环境下 100% 覆盖率几乎不可能达成。旧公式 95% 时仍给 0.60 分，无法有效区分优劣。

---

## 核心算法

| 组件 | 选用方案 |
|------|----------|
| 区域分解 | Sweep 扫描线分解 |
| 条带排序 | Boustrophedon (来回扫描) |
| 掉头规划 | Direct Route 折线（控制点保真；待 VM 验证） |
| TSP 路由 | OR-Tools genRoute |
| 路径简化 | RDP (Ramer-Douglas-Peucker) |
| 覆盖评估 | 网格采样法 |
| 边界处理 | Headland + fillBoundaryGaps |

---

## 已知问题

| 问题 | 优先级 | 状态 | 方案 |
|------|:--:|:--:|------|
| S7 剩余 0.19% 未覆盖（物理不可达空隙） | P1 | 已定位 | 空隙分类器确认 15 处物理不可达（0.59m²） |
| 候选清洁宽度 0.90 m 尚未做连续清洁痕迹验证 | P1 | 待实测 | 实测有效连续清洁带后再冻结产品参数 |
| cell 接缝连续漏扫带 | P1 | 待优化 | swath 间距/重叠优化 |
| A* 自由空间路由（替代 boundary-walk） | P0 | 开发中 | v9.9 启动，VM 崩溃待排查 |
| /tmp/ 硬编码路径 → 平台临时目录 | P2 | 已修复 | `artifact_sink` 模块 (v9.7) |
| 线程安全（零 mutex） | P2 | 已知 | 单线程下安全，切多线程时处理 |

---

## 变更日志

| 日期 | 关键事件 |
|------|----------|
| 07-07 | 项目初始化，F2C 原生流程 |
| 07-08 | direct 掉头、自适应 headland |
| 07-11 | 去重移除：S4 75%→97.4% |
| 07-12 | Sweep 分解：S3 42→7 cells |
| 07-14 | v8 稳定版：评分重构 + Cell连接优化 |
| 07-15 | v9.2.1：贪心排序 + 边界补线修复 |
| 07-16 | v9.3：Codex PathState 语义修正 + P0/P1 清仓 + 全项目审查；GitHub 首次推送 |
| 07-17 | v9.6~v9.7：5 个 Codex 修复 (S4近重合/门控/S3去重/tmp) + 交叉审计；47/47 gtest + 7/7 GATE PASS |
| 07-18 | v9.7 正式验收：7/7 场景 99.99% 覆盖率，平均 86.1 分 |
| 07-19 | v9.8 C-space退役 + 空隙分类器 + Phase 4A；v9.9 filter/merge顺序修复 (S7 +0.35%/+6.7分) |
| **07-20** | **v9.10 S8重命名 + v9.11 穿洞硬约束修复；8/8 场景 hole_crossings=0，平均 99.98%/81.7** |

---

> 详细文档见 `docs/F2C_技术文档.md`
> 每日日志见 `docs/daily_logs/`
