---
name: path-planning-algorithms
description: >
  覆盖路径规划算法知识库。触发条件：路径规划算法、Boustrophedon 分解、Ramer-Douglas-Peucker、
  RDP 简化、凸包/凹包、Delaunay 三角剖分、TSP 路径排序、Dubins 曲线、Reeds-Shepp、
  A* 路径规划、区域分解、子区域过滤、Swath 多角度优化。适用于扫地机器人、农机、无人机
  全覆盖作业规划。
---

# 覆盖路径规划算法专家

你是移动机器人全覆盖路径规划 (CCPP: Complete Coverage Path Planning) 的算法专家。

## 1. Boustrophedon（牛耕式）分解

### 原理
将多边形沿扫线方向分解为多个梯形/三角形子区域，每个子区域内用来回往复线覆盖。

### 关键步骤
1. 确定扫线方向角 θ
2. 对多边形顶点按垂直于 θ 的方向排序
3. 扫描线扫描时检测临界点（多边形顶点处拆分）
4. 生成子区域列表

### 代码伪码
```
function boustrophedon_decomp(polygon, angle):
    # 旋转多边形使扫线方向与 x 轴对齐
    rotated = rotate(polygon, -angle)
    # 获取所有顶点，按 x 坐标排序
    vertices = sorted(rotated.vertices, key=lambda v: v.x)
    # 从左到右扫描
    cells = []
    for each critical x in vertices:
        # 用垂直线切割多边形
        # 检测拓扑变化（出现/消失/分裂/合并）
        # 生成子多边形
    return cells
```

### 优化方向
- 多角度尝试：对 [0°, 180°] 内多个角度分别生成分解，选转弯次数最少的
- 子区域合并：相邻同方向子区域可合并减少掉头

## 2. RDP (Ramer-Douglas-Peucker) 路径简化

### 原理
递归地简化折线：找离首尾连线最远的点，若距离 > 阈值则保留并递归。

### 伪代码
```
function rdp(points, epsilon):
    dmax = 0, index = 0
    for i in 1..len(points)-2:
        d = perpendicular_distance(points[i], points[0], points[-1])
        if d > dmax: dmax = d, index = i
    if dmax > epsilon:
        left = rdp(points[0..index], epsilon)
        right = rdp(points[index..-1], epsilon)
        return left[:-1] + right
    else:
        return [points[0], points[-1]]
```

### 参数选择
- `epsilon` 太小 → 简化不够，路径点过多
- `epsilon` 太大 → 路径变形，可能遗漏覆盖区域
- 建议：取 robot_width × 0.1 ~ 0.3

## 3. Dubins 曲线

### 参数
- 转弯半径 r（由车辆最小转弯半径决定）
- 起点位姿 (x1, y1, θ1) → 终点位姿 (x2, y2, θ2)

### 路径类型
6 种可能：LSL, LSR, RSL, RSR, LRL, RLR
(L = 左转, R = 右转, S = 直行)

### 选择策略
- 取 6 种中路径最短的
- 对于覆盖路径：相邻 swath 之间用 U 型掉头

## 4. TSP 路径排序

### 问题
多个子区域 → 需要决定访问顺序使总路径最短

### 方案对比
| 方案 | 复杂度 | 精度 |
|------|--------|------|
| 最近邻贪心 | O(n²) | 近似解 |
| OR-Tools TSP | O(2^n) 启发式 | 近似最优 |
| 2-opt 局部优化 | O(n²) per iter | 改进现有解 |
| 模拟退火 / GA | 取决于参数 | 可调 |

## 5. 子区域过滤

### 过滤条件
- **面积过滤**：面积 < 阈值（如 robot_width² × 2）的子区域 → 合并或丢弃
- **形状过滤**：长宽比极端（如 > 10:1）的子区域 → 特殊处理（单线路径而非往复）
- **可达性过滤**：被障碍物完全包围的子区域 → 跳过

## 6. Swath 多角度策略

### 策略
1. 计算多边形最小宽度方向（最佳角度）
2. 在最佳角度 ±30° 内采样 3-5 个角度
3. 对每个角度生成完整路径，计算总长度 + 转弯次数
4. 选综合最优角度

### 评估函数
```
score = path_length + turn_penalty × num_turns
```
其中 `turn_penalty` = 掉头时间 × 行进速度（把时间折算成等价距离）
