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

## ROS2 集成注意事项

- Fields2Cover 为 C++ 库，需要 CMakeLists.txt 中 `find_package(fields2cover REQUIRED)`
- 路径点坐标通过 ROS2 topic 发布：`nav_msgs::msg::Path` 或自定义消息
- 可视化用 `F2CVisualizer` 导出图片，不必依赖 rviz2

## 调试技巧

- 每个阶段独立可视化：先看 swath 对不对，再看 headland，最后看完整路径
- 输出 swath 数量、总路径长度、转弯次数到 ROS2 logger
- 用 `F2CVisualizer::plot()` 导出中间结果到 PNG
