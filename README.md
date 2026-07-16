# F2C 全覆盖路径规划系统

> **版本**: v9.4 (2026-07-16) | **编译**: 0 error 0 warning | **测试**: 11/11 ✅
> 
> *v9.4 = v9.3 + Route 分组修复 + direct 控制点保留 + 孔洞闭环 + 实车参数 0.75/0.90*

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

## 当前基准（v9.4，batch_0716_1528，实测参数）

> `robot_width=0.75`（实车）、`coverage_width=0.90`（名义值，待直线清扫痕迹实测确认）。notched 的 0.90m 宽度在含孔洞区域偏粗，待调优。

| 场景 | 覆盖率 | 得分 | 路径长 | 路径点 | 耗时 |
|------|:------:|:----:|:-----:|:-----:|:---:|
| S1 矩形 20×15m | **100.00%** | 81.1 | 389.3m | 40 | 94ms |
| S2 L 形 | **100.00%** | 80.2 | 525.4m | 70 | 206ms |
| S3 含孔洞 | 99.77% | 78.1 | 598.5m | 202 | 573ms |
| S4 窄走廊 | **100.00%** | 65.1 | 80.8m | 10 | 19ms |
| S5 不规则 | **100.00%** | 73.8 | 134.4m | 44 | 79ms |
| S6 多区域 | 99.83% | 65.6 | 147.6m | 96 | 198ms |
| notched 缺口孔 | 95.92% | 20.6 | 422.0m | 140 | 373ms |
| **平均** | **99.36%** | **66.4** | — | — | — |

> 路径点、路径长较旧参数均减少 40%+。notched 需单独处理：0.90m 宽度在孔洞窄通道处过粗。

## 缺陷修复回归（batch_0716_1522，旧参数 0.95/0.45）

> 验证三项代码缺陷修复，S6 越界从 8.60m→1.00m，S3/notched 越界消失，孔洞误报清零。

| 场景 | 覆盖率 | 得分 | vs 修复前 |
|------|:------:|:----:|:--:|
| S6 多区域 | 98.75% | 71.1 | +8.9 |
| notched | 98.83% | 77.1 | +5.1 |
| S3 含孔洞 | 99.24% | 79.6 | +0.6 |

## 历史基准（v9.3，旧参数）

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

| 问题 | 优先级 | 方案 |
|------|:--:|------|
| Route 边界补线插入顺序错误，产生跨区/越界长连接 | P0 | 本地已修 `59d6418`；待 VM 场景验证 |
| route waypoint 绕行折线被 `simplifyConnection` 拉成弦线 | P0 | 本地已修 `15aa918`；direct 保留全部 Route 控制点并跳过 RDP，待 VM 验证 |
| 孔洞交叉诊断未闭环，S3/S6/notched 产生误报 | P1 | 本地已修 `548070e`；统一闭环构造及点包含测试，待 VM 验证 |
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
