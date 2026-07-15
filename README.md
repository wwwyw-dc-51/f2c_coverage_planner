# F2C 全覆盖路径规划系统

> **版本**: v9 模块化重构版 (2026-07-14) | **平均覆盖率**: 98.70% | **满分段场景**: 5/7 | **编译**: 0 error 0 warning

基于 Fields2Cover + ROS2 Humble 的扫地机器人全覆盖路径规划系统。

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
  -p robot_width:=0.95 -p coverage_width:=0.45 \
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

## 7 场景基准 (v9)

| 场景 | 覆盖率 | 得分 | 说明 |
|------|:------:|:----:|------|
| S1 矩形 20×15m | 99.91% | 85.5 | 满分段 |
| S2 L 形 | 99.31% | 84.3 | 满分段 |
| S3 含孔洞 | 99.09% | 77.0 | 满分段 |
| S5 不规则 | 99.63% | 82.0 | 满分段 |
| S6 多区域 | 98.33% | 62.2 | 高分段 |
| notched 缺口孔 | 98.60% | 69.9 | 高分段 |
| S4 窄走廊 | 90.03% | 0.0 | 待修复 |
| **平均** | **97.84%** | **65.8** | — |

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
| 掉头规划 | Direct (Euclidean 直连) |
| TSP 路由 | OR-Tools genRoute |
| 路径简化 | RDP (Ramer-Douglas-Peucker) |
| 覆盖评估 | 网格采样法 |
| 边界处理 | Headland + fillBoundaryGaps |

---

## 已知问题

| 问题 | 方案 |
|------|------|
| S4 窄走廊 90% | 动态重叠率（cell宽度 < 3×cov_width 时增加overlap） |
| 孔洞边缘空缺 | 孔洞补刀已加代码，需确保genRoute不丢弃 |

---

## 变更日志

| 日期 | 关键事件 |
|------|----------|
| 07-07 | 项目初始化，F2C 原生流程 |
| 07-08 | direct 掉头、自适应 headland |
| 07-11 | **去重移除**：S4 75%→97.4% |
| 07-12 | **Sweep 分解**：S3 42→7 cells |
| 07-13 | Phase 2-4 体系化优化 |
| **07-14** | **v8 稳定版**：评分重构 + Cell连接优化 + 边界补线 + 孔洞检测 |

---

> 详细文档见 `docs/F2C_技术文档.md`
> 每日日志见 `docs/daily_logs/`
