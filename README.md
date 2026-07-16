# F2C 全覆盖路径规划系统

> **版本**: v9.3 (2026-07-16) | **S4 覆盖率**: 99.95% | **编译**: 0 error 0 warning | **测试**: 5/5 ✅
> 
> *v9.3 = v9.2.1 贪心排序 + Codex PathState 语义修正 + P0/P1 清仓*

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
│   ├── test_polygons/                 # S1~S6 测试多边形
│   └── config/f2c_areas/             # 实际场景多边形
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

## 历史 7 场景算法基准（v9.3，batch_0716_1234，旧物理参数）

> 本表使用 `robot_width=0.95`、`coverage_width=0.45`，仅用于追踪算法回归。`batch_0716_1331` 暴露了 Route 插入和连接拉直缺陷，也仍使用旧参数；修复缺陷并按候选 `0.75/0.90` 重跑前，当前没有可作为产品验收依据的新基线。

| 场景 | 覆盖率 | 得分 | 路径长 | 耗时 | 说明 |
|------|:------:|:----:|:-----:|:---:|------|
| S1 矩形 20×15m | 99.92% | 82.1 | 742.7m | 292ms | ✅ |
| S2 L 形 | 99.60% | 83.9 | 941.5m | 653ms | ✅ |
| S3 含孔洞 | 99.09% | 75.7 | 1143.5m | 2767ms | ✅ |
| S4 窄走廊 | **99.95%** | **76.0** | 120.4m | 27ms | 🎉 v9.2修复 |
| S5 不规则 | 99.63% | 77.5 | 242.1m | 276ms | ✅ |
| S6 多区域 | 98.44% | 62.2 | 232.0m | 352ms | ✅ |
| notched 缺口孔 | 98.62% | 67.2 | 770.6m | 1398ms | ✅ |
| **平均** | **99.32%** | **74.9** | — | — | — |

> 对比 v9: 覆盖率 +1.48%，得分 +9.1。S4 从 90%→99.95%（PathState 语义修正）。

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
| 掉头规划 | Direct 选项（当前 connection 仍会进入 F2C 简化，待修） |
| TSP 路由 | OR-Tools genRoute |
| 路径简化 | RDP (Ramer-Douglas-Peucker) |
| 覆盖评估 | 网格采样法 |
| 边界处理 | Headland + fillBoundaryGaps |

---

## 已知问题

| 问题 | 优先级 | 方案 |
|------|:--:|------|
| Route 边界补线插入顺序错误，产生跨区/越界长连接 | P0 | 使用 `addConnectedSwaths`，保证 connection 位于新 group 之前 |
| route waypoint 绕行折线被 `simplifyConnection` 拉成弦线 | P0 | direct 模式保留控制点；弦线替换前做可行域验证 |
| 孔洞交叉诊断未闭环，S3/S6/notched 产生误报 | P1 | 统一闭环构造 hole ring，并增加点包含测试 |
| 候选清洁宽度 0.90 m 尚未做连续清洁痕迹验证 | P1 | 实测有效连续清洁带后再冻结产品参数 |
| 边界补线重复/折返（两套系统同时生效） | P1 | 合并到 genRoute 前，无方向去重 |
| cell 接缝连续漏扫带 | P1 | swath 间距/重叠优化 |
| S4 重叠率 29.3%（40m² 走了 120m） | P1 | 动态重叠率 `N=ceil(W/C), spacing=(W-C)/(N-1)` |
| /tmp/ 硬编码路径 | P2 | Windows 兼容 |
| 线程安全（零 mutex） | P2 | 单线程下安全，切多线程时处理 |

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
| **07-16** | **v9.3：Codex PathState 语义修正 + P0/P1 清仓 + 全项目审查** |
| | S4 90%→99.95%，batch_test exit code ✅，5/5 测试通过 |
| | GitHub 首次推送，仓库 wwwyw-dc-51/f2c_coverage_planner |

---

> 详细文档见 `docs/F2C_技术文档.md`
> 每日日志见 `docs/daily_logs/`
