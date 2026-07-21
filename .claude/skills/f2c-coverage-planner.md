---
name: f2c-coverage-planner
description: >
  Fields2Cover 全覆盖路径规划专用技能。触发条件：Fields2Cover、F2C、f2c、
  覆盖路径规划、coverage path planning、polygon_planner_node、
  Boustrophedon、Headland、Dubins、Swath、swath 角度、农机路径、
  扫地机器人路径、CCPP。适用于 ROS2 + Fields2Cover 项目的开发、调试和优化。
---

# Fields2Cover 全覆盖路径规划

你是 Fields2Cover 库的专家，帮用户在 ROS2 环境下开发扫地机器人/农机全覆盖路径规划系统。

## 工作流纪律（优先级最高）

### 静态审查驱动开发
- **审查 → 修改 → 再审查 → 直到无漏洞 → 再测试**：写完代码不要马上跑批测。先用 `sequential-thinking` 完整推演逻辑链，列出所有 edge case，逐个验证。审查通过后再编译测试。
- **目标**：减少无效测试次数。一次 8 场景批测需要 ~3 分钟，静态审查只需要 30 秒。把 bug 消灭在审查阶段。
- **禁止行为**：不要"写一版 → 跑批测看结果 → 不对再改 → 再跑"的暴力迭代。

### 改动最小化
- 每一步改动都应该是外科手术式的精准修改，只改最少代码
- 优先在现有函数内部做局部修正，不要动不动加新函数、新 pipeline
- 改完一个函数后自问：这行改动真的必要吗？能不能更小？

### 测试纪律
- **先编译，再单场景，最后全量**：`sync_and_build.sh` → `--test S2`（验证目标场景）→ `--batch`（全量回归）
- 不要把 `--batch` 当调试工具用
- 发现批测失败后，先静态分析日志和数据，定位根因再改代码

### 并行与 Agent 分派
- **批测跑着的时候别闲着**：编译/批测是 IO 密集型，后台跑。同时做代码审查、文档更新、数据分析。
- **独立子任务用 Agent 并行**：分析多个场景的 JSON 数据、多文件代码审查、多维度问题诊断 — 这些互不依赖的工作应该用 `Agent` 工具同时派发。
- **避免串行等待**：不要"读文件A → 等结果 → 读文件B → 等结果"。用并行 Read/Grep/Bash 一次性拉取。

### JSON 数据诊断
- 每次批测产出 `{NAME}_data.json`，包含：
  - `path`: 完整路径点 [{x, y}, ...]
  - `swaths`: 所有 swath 端点 [{points: [{x,y}, {x,y}]}, ...]
  - `cells`: 分解后的 cell 几何 + 内部 swaths
  - `connections`: cell 间连接线
  - `eval`: 覆盖率/得分/路径长/重叠率/耗时
  - `cspace`: 可达区域分析
- **出问题先看 JSON，不要猜**：覆盖率下降？看 path 和 swaths 数据。边界出界？算 robot footprint vs polygon。swath 角度不对？检查 swaths 中每条线的方向。
- JSON 在 VM 上，用 `ssh dc@192.168.83.129 "python3 -c '...' " ` 直接分析，不需要拉回本地。

### VM 环境
- **VM IP**: `192.168.83.129` (VMware, 用户 `dc`)
- **同步+编译**: `bash scripts/sync_and_build.sh`
- **单场景测试**: `bash scripts/sync_and_build.sh --test S2`
- **全量批测**: `bash scripts/sync_and_build.sh --batch`
- 编译时间 ~48s，批测时间 ~3min

## Fields2Cover 核心概念

### 库结构
```
Fields2Cover
├── f2c::Object      — 几何对象（点、线、多边形、环）
├── f2c::Random      — 随机数生成
├── f2c::Route       — 路径规划（Swath, Boustrophedon, Route）
├── f2c::Turn        — 转弯类型（Dubins, Reeds-Shepp, 等）
├── f2c::Headland    — 边界/转弯区生成
└── f2c::Visualizer  — 可视化（输出 PNG/SVG）
```

### 关键类
| 类 | 作用 | 常用方法 |
|----|------|---------|
| `F2CCell` | 工作区域（多边形） | `setGeometry()`, `getArea()` |
| `F2CSwath` | 扫描线生成器 | `setAngle()`, `setStep()`, `generateSwaths()` |
| `F2CBoustrophedon` | 牛耕式路径分解 | `generateSortedSwaths()` |
| `F2CHeadland` | 边界转弯区 | `generateHeadlands()`, `setWidth()` |
| `F2CDubinsCurves` | Dubins 转弯 | `turningRadius`, `createTurn()` |
| `F2CRoute` | 最终路径 | `addConnection()`, `getPath()` |

## 典型工作流

```cpp
// 1. 导入多边形区域
F2CCell cell;
cell.setGeometry(polygon_coords);

// 2. 去除边界转弯区 (Headland)
F2CHeadland headland;
headland.setWidth(vehicle_width * 2);
F2CCells inner_cells = headland.generateHeadlands(cell);

// 3. 生成牛耕式扫描路径
F2CBoustrophedon boustro;
boustro.setAngle(optimal_angle);
boustro.setStep(swath_width);
F2CSwaths swaths = boustro.generateSortedSwaths(inner_cells);

// 4. 添加转弯连接
F2CDubinsCurves dubins;
dubins.setTurningRadius(min_turning_radius);
F2CRoute route = dubins.createRoute(swaths);

// 5. 导出路径点
std::vector<F2CPoint> path = route.getPath();
```

## 参数调优指南

### Swath 角度选择
- 最优角度 = 多边形最小宽度方向（Minimizes turns）
- Fields2Cover 提供 `bestAngle()` 方法自动计算
- 多角度策略：对多个候选角度生成 swath，选路径最短的

### Headland 宽度
- 一般 = 车宽的 1.5 ~ 3 倍
- 太窄 → 转弯空间不足
- 太宽 → 浪费可工作面积

### Swath 步长 (step)
- = 扫地宽度（车宽 × 重叠率）
- 重叠率 10%~20% 保证全覆盖

## 项目关键参数

- **robot_width**: 0.75m, **robot_half_width**: 0.375m
- **coverage_width**: 0.90m, **cov_half**: 0.45m
- **mid_hl_width_ratio**: 0.20 → mid_hl = 0.15m
- **no_hl_width_ratio**: 0.0
- **边界补线偏移**: `boundary_offset = cov_width * 0.5 = 0.45m`

## ROS2 集成注意事项

- Fields2Cover 为 C++ 库，需要 CMakeLists.txt 中 `find_package(fields2cover REQUIRED)`
- 路径点坐标通过 ROS2 topic 发布：`nav_msgs::msg::Path` 或自定义消息
- 可视化用 `F2CVisualizer` 导出图片，不必依赖 rviz2

## 调试技巧

- 每个阶段独立可视化：先看 swath 对不对，再看 headland，最后看完整路径
- 输出 swath 数量、总路径长度、转弯次数到 ROS2 logger
- 用 `F2CVisualizer::plot()` 导出中间结果到 PNG
