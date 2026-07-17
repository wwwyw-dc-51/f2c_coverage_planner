# F2C 开发速查 — 给 Codex 的参考

> 从 Claude 的 f2c-coverage-planner / path-planning-algorithms 两个 Skill 中提炼的工程要点。

## 1. Fields2Cover 库结构

```
Fields2Cover
├── f2c::Object      — 几何对象（点、线、多边形、环）
├── f2c::Random      — 随机数生成
├── f2c::Route       — 路径规划（Swath, Boustrophedon, Route）
├── f2c::Turn        — 转弯类型（Dubins, Reeds-Shepp 等）
├── f2c::Headland    — 边界/转弯区生成
└── f2c::Visualizer  — 可视化（输出 PNG/SVG）
```

## 2. 关键类速查

| 类 | 作用 | 常用方法 |
|----|------|---------|
| `F2CCell` | 工作区域（多边形） | `setGeometry()`, `getArea()` |
| `F2CSwath` | 扫描线生成器 | `setAngle()`, `setStep()`, `generateSwaths()` |
| `F2CBoustrophedon` | 牛耕式路径分解 | `generateSortedSwaths()` |
| `F2CHeadland` | 边界转弯区 | `generateHeadlands()`, `setWidth()` |
| `F2CDubinsCurves` | Dubins 转弯 | `turningRadius`, `createTurn()` |
| `F2CRoute` | 最终路径 | `addConnection()`, `getPath()` |

## 3. 典型工作流

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

## 4. 参数调优

- **Swath 角度**：最优角度 = 多边形最小宽度方向，可用 `bestAngle()` 自动计算；多角度策略在最佳角度 ±30° 采样 3-5 个角度
- **Headland 宽度**：车宽的 1.5~3 倍；太窄转弯空间不足，太宽浪费面积
- **Swath 步长**：= 扫地宽度，重叠率 10%~20% 保证全覆盖

## 5. 本项目物理参数

- `robot_width` = 0.75 m（实测）
- `coverage_width` = 0.90 m（实测）
- 预设重叠率 = 3%，有效行距 w_eff = 0.873 m
- `turn_merge_distance` = max(0.75, 1.05 × w_eff) = 0.91665 m
- 最小转弯半径 = 1.0 m

## 6. 本项目关键算法模块

### Hole Repair（`path_planner.cpp`）
- `makeAvoidanceRings()` — GEOS buffer 生成孔洞避让环
- `repairRouteConnectionsAroundHoles()` — 生成路径后修复穿越孔洞的连接线
- `findHoleIntersections()` — 检测线段与孔洞的交点
- `walkHoleBoundary()` — 沿孔洞边界绕行
- `repairConnectionPolyline()` — 修复单条穿越孔洞的连接

### 评分公式（`coverage_evaluator.hpp`）
- 覆盖门控：`G = clamp((c - 0.90) / 0.09, 0, 1)³`
- 效率分 = 0.33×work_ratio + 0.27×distance + 0.15×turns + 0.15×overlap + 0.10×time
- 总分 = G × efficiency × 100
- 通过阈值：99.5%

### 路径结构
- Route = swath groups + connections
- `planDirectPath()` → PathState 序列 → `materializePath()` → 最终坐标点
- 路径类型：SWATH / TURN

## 7. 算法知识速查

### Boustrophedon 分解
- 将多边形按扫线方向分解为梯形子区域
- 关键步骤：旋转 → 顶点排序 → 扫描线检测临界点 → 生成子区域

### Dubins 曲线
- 6 种路径：LSL, LSR, RSL, RSR, LRL, RLR
- 取 6 种中最短，覆盖路径中相邻 swath 用 U 型掉头

### RDP 路径简化
- epsilon 建议：robot_width × 0.1 ~ 0.3
- 递归找最远点，距离 > epsilon 则保留并递归

### TSP 子区域排序
- 方案：最近邻贪心 O(n²) / OR-Tools TSP 近似最优 / 2-opt 局部优化

### 评估函数（角度选择）
```
score = path_length + turn_penalty × num_turns
```
turn_penalty = 掉头时间 × 行进速度（时间折算为等价距离）

## 8. ROS2 注意

- CMakeLists.txt 需要 `find_package(fields2cover REQUIRED)`
- 路径通过 `nav_msgs::msg::Path` 发布
- 可视化用 `F2CVisualizer` 导出 PNG，不依赖 rviz2
- 当前 ROS2 版本：Humble

## 9. 测试与批处理

- 单元测试：`src/yingshi_robot/test/`，用 `colcon test`
- 批处理脚本：`scripts/batch_test_v2.sh`
- 评分公式测试：`coverage_evaluator_test.cpp`（7 个测试）
- 路径不变性测试：`planning_invariants_test.cpp`（5 个测试）
- 当前状态：21/21 测试通过，7/7 场景通过
