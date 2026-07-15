# Swath 角度优化参考资料

> 收集日期：2026-07-07
> 目的：嵌入式场景下，用数学剪枝替代暴力枚举，保证最优的同时节省算力

---

## 核心定理：最优 Swath 角度一定平行于多边形的某条边

### 来源：Rotating Calipers 算法 (Shamos 1978, Toussaint 1983)

**定理陈述**：凸多边形的最小宽度（minimum width）总是由一对平行支撑线确定，其中**至少一条支撑线与多边形的一条边重合**。

**证明概要**：
1. 考虑多边形两侧的任意一对平行支撑线
2. 如果**两条线都不与边重合**，则总可以将它们**略微旋转**以**减小**间距
3. 因此，最小宽度配置必须至少有一条支撑线与边重合
4. 搜索空间从无穷多个方向缩减为**最多 N 个方向**（每条边一个）

**推论（用于覆盖路径规划）**：
Swath 条数 = 多边形在 swath 垂直方向上的宽度 / 覆盖宽度。最小化 swath 条数等价于最小化投影宽度。因此最优 swath 方向一定**平行于**多边形的某条边。

### 参考文献
- Shamos, M.I. (1978). *Computational Geometry.* Ph.D. dissertation, Yale University
- Toussaint, G.T. (1983). *Solving geometric problems with the rotating calipers.* Proc. MELECON '83
- Houle, M.E. & Toussaint, G.T. (1988). *Computing the width of a set.* IEEE Trans. PAMI, Vol. 10, No. 5, pp. 761–765
- Wikipedia: https://en.wikipedia.org/wiki/Rotating_calipers

---

## Fields2Cover 内置的角度优化方法

### 1. BruteForce (`f2c::sg::BruteForce`)
- 以 `step_angle` 为步长遍历 360° 所有角度
- `computeBestAngle()` 函数内部调用 `generateBestSwaths()`
- 保证全局最优，但计算量大（O(360/step_angle × 顶点数)）
- 支持多线程并行

### 2. Oksanen 方法
- 来源：Oksanen & Visala (2006/2007)
- 两阶段搜索：
  - 第一阶段：识别一组有希望的角度方向
  - 第二阶段：在最佳候选附近进行贪心搜索
- 比 BruteForce 迭代次数少，但**不保证全局最优**

### 3. 最长边方向（Longest Edge, LE）
- 基线方法，swath 平行于地块最长边
- 农民常用的经验法则
- O(n) 计算量

### 参考文献
- Mier, Valente, de Bruin (2023). *Fields2Cover: An Open-Source Coverage Path Planning Library for Unmanned Agricultural Vehicles.* IEEE RA-L. DOI: 10.1109/LRA.2023.3239308
- Fields2Cover 论文: https://arxiv.org/html/2210.07838
- Fields2Cover 源码 (BruteForce): https://docs.ros.org/en/api/fields2cover/html/brute__force_8cpp_source.html
- Oksanen, T. & Visala, A. (2007). *Path Planning Algorithms for Agricultural Machines.* CIGR Ejournal Vol. IX

---

## 实用算法：用多边形边缘方向替代暴力枚举

### 算法复杂度对比

| 方法 | 角度候选数 | 每次评估 | 总复杂度 | 最优性 |
|------|-----------|---------|---------|--------|
| BruteForce (step=5°) | 72 | O(v) | O(72v) | 近似（步长限制） |
| BruteForce (step=1°) | 360 | O(v) | O(360v) | 近似（步长限制） |
| 写死候选 "0,45,90,135" | 4 | O(v) | O(4v) | 可能漏掉最优 |
| **边缘方向（本文推荐）** | **N（边数）** | O(v) | O(N×v) | **数学保证最优** |
| Rotating Calipers | N | O(1) | O(N) | 仅对凸多边形 |

### 实现细节

1. 获取多边形顶点环
2. 计算每条边的方向角：`atan2(y2-y1, x2-x1)`
3. 去重（合并 2° 以内的相近角度）
4. 每个唯一边缘角度作为候选，生成 swath 后选条数最少的
5. 对于非凸多边形，先取凸包，或在 Boustrophedon 分解后的每个子 cell 上分别计算

### 伪代码

```
extractEdgeAngles(polygon):
    for each edge (vi, vi+1) in polygon:
        angle = atan2(vi.y - vi+1.y, vi.x - vi+1.x)
        angles.add(angle)
    return deduplicate(angles, tolerance=2°)

optimizeSwathAngle(cell, swath_generator):
    candidates = extractEdgeAngles(cell)
    best_swaths = null, best_count = INF
    for angle in candidates:
        swaths = swath_generator.generateSwaths(angle, ...)
        if swaths.size() < best_count:
            best_swaths = swaths
            best_count = swaths.size()
    return best_swaths
```

---

## 实现计划

- [ ] 新增 `extractEdgeAngles()` 函数到 `polygon_planner_node.cpp`
- [ ] 修改 `optimizeSwathAngle()` 使用边缘方向替代写死的角度候选
- [ ] 保留 `swath_angle_candidates` 参数作为额外候选（可与边缘方向合并）
- [ ] 保留 `computeCellMainDirection()`（最长边方向）作为回退方案
