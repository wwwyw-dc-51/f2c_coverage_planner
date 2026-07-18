#!/usr/bin/env python3
"""F2C 覆盖规划可视化渲染脚本"""
import sys, yaml, json, math, os, glob
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPolygon
from matplotlib.path import Path as MPath
import numpy as np

def render(scenario_name, outer, holes, path_pts, eval_result, output_png,
           grid_json=None, component_paths=None):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # --- 左图: 路径规划图 ---
    ax1.set_title(f'{scenario_name} - Planned Path', fontsize=12)
    poly = MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=2)
    ax1.add_patch(poly)
    for i, h in enumerate(holes):
        hp = MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1.5, alpha=0.7)
        ax1.add_patch(hp)

    path_groups = component_paths or ([path_pts] if path_pts else [])
    for path_group in path_groups:
        px = [p['x'] for p in path_group]
        py = [p['y'] for p in path_group]
        ax1.plot(px, py, color='darkorange', linewidth=0.8, alpha=0.9)

    ax1.set_aspect('equal')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')

    # --- 右图: 覆盖热力图 ---
    ax2.set_title(f'{scenario_name} - Coverage Map', fontsize=12)
    poly2 = MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=1.5)
    ax2.add_patch(poly2)
    for h in holes:
        hp2 = MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1, alpha=0.7)
        ax2.add_patch(hp2)

    # 尝试从 C++ 输出的 .grid.json 读取网格数据（避免重算）
    covered_x, covered_y = [], []
    uncovered_x, uncovered_y = [], []

    grid_files = grid_json if isinstance(grid_json, list) else [grid_json]
    grid_files = [path for path in grid_files if path and os.path.exists(path)]
    if grid_files:
        covered_points = set()
        uncovered_points = set()
        for grid_file in grid_files:
            with open(grid_file) as f:
                gd = json.load(f)
            covered_points.update(map(tuple, gd.get('covered', [])))
            uncovered_points.update(map(tuple, gd.get('uncovered', [])))
        uncovered_points.difference_update(covered_points)
        covered_x = [p[0] for p in covered_points]
        covered_y = [p[1] for p in covered_points]
        uncovered_x = [p[0] for p in uncovered_points]
        uncovered_y = [p[1] for p in uncovered_points]
        print(f'  [render] Using C++ grid data: {len(covered_x)} covered + {len(uncovered_x)} uncovered')
    else:
        # 回退：Python 自行网格采样
        res = 0.10
        cov_width = 0.90
        half_w = cov_width / 2.0

        ox, oy = zip(*outer)
        min_x, max_x = min(ox), max(ox)
        min_y, max_y = min(oy), max(oy)

        poly_path = MPath(outer)
        hole_paths = [MPath(h) for h in holes]

        def in_target(px, py):
            if not poly_path.contains_point((px, py)):
                return False
            for hp in hole_paths:
                if hp.contains_point((px, py)):
                    return False
            return True

        segment_rows = [
            (group[i]['x'], group[i]['y'],
             group[i + 1]['x'], group[i + 1]['y'])
            for group in path_groups
            for i in range(len(group) - 1)
        ]
        if segment_rows:
            segs = np.array(segment_rows)
        else:
            segs = np.zeros((0, 4))

        def is_covered(px, py):
            if len(segs) == 0:
                return False
            x1, y1, x2, y2 = segs[:,0], segs[:,1], segs[:,2], segs[:,3]
            dx = x2 - x1; dy = y2 - y1
            len2 = dx*dx + dy*dy
            mask = len2 > 1e-12
            t = np.zeros_like(len2)
            t[mask] = np.clip(((px-x1[mask])*dx[mask] + (py-y1[mask])*dy[mask]) / len2[mask], 0, 1)
            proj_x = x1 + t*dx; proj_y = y1 + t*dy
            dists = np.sqrt((px-proj_x)**2 + (py-proj_y)**2)
            return np.min(dists) <= half_w

        for gx in np.arange(min_x, max_x, res):
            for gy in np.arange(min_y, max_y, res):
                if in_target(gx, gy):
                    if is_covered(gx, gy):
                        covered_x.append(gx); covered_y.append(gy)
                    else:
                        uncovered_x.append(gx); uncovered_y.append(gy)

    if covered_x:
        ax2.scatter(covered_x, covered_y, c='green', s=1, alpha=0.5, marker='s', label='Covered')
    if uncovered_x:
        ax2.scatter(uncovered_x, uncovered_y, c='red', s=2, alpha=0.7, marker='s', label='Uncovered')

    ax2.set_aspect('equal')
    ax2.set_xlabel('X (m)')
    cov_raw = eval_result.get('coverage_rate', 0) or 0
    # 自适应格式：>1.0 已是百分比  |  ≤1.0 是比率需×100
    cov_rate = cov_raw if cov_raw > 1.0 else cov_raw * 100.0
    score = eval_result.get('single_score', 0) or 0
    ax2.text(0.02, 0.98, f'Coverage: {cov_rate:.1f}% | Score: {score:.1f}',
             transform=ax2.transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    plt.savefig(output_png, dpi=120, bbox_inches='tight')
    plt.close()
    print(f'  Rendered: {output_png}')


def render_cells(scenario_name, outer, holes, cells_data, output_png):
    """Cell 划分图：每个 cell 用不同颜色填充"""
    import matplotlib.cm as cm
    import numpy as np
    from matplotlib.patches import Polygon as MPolygon

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_title(f'{scenario_name} - Cell Decomposition', fontsize=12)

    # 外边界 + 孔洞
    poly = MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=2)
    ax.add_patch(poly)
    for h in holes:
        hp = MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1.5, alpha=0.7)
        ax.add_patch(hp)

    # 每个 cell 用不同颜色
    colors = cm.tab10(np.linspace(0, 1, max(len(cells_data), 1)))
    for ci, cell in enumerate(cells_data):
        boundary = cell.get('boundary', [])
        if len(boundary) < 3:
            continue
        color = colors[ci % len(colors)]
        cp = MPolygon(boundary, fill=True, facecolor=color, edgecolor='black',
                      linewidth=1.5, alpha=0.6)
        ax.add_patch(cp)
        # 标注 cell ID
        cx = sum(p[0] for p in boundary) / len(boundary)
        cy = sum(p[1] for p in boundary) / len(boundary)
        ax.text(cx, cy, f'C{ci}', fontsize=9, ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
        # 入口点（绿色圆，白边，大号）
        if 'entry' in cell:
            ax.plot(cell['entry']['x'], cell['entry']['y'], 'o', color='#00cc44',
                    markersize=14, markeredgewidth=2, markeredgecolor='white', zorder=10)
        # 出口点（红色方块，白边，大号）
        if 'exit' in cell:
            ax.plot(cell['exit']['x'], cell['exit']['y'], 's', color='#e83015',
                    markersize=14, markeredgewidth=2, markeredgecolor='white', zorder=10)

    # 按多边形包围盒裁剪视图，避免大空白
    ox = [p[0] for p in outer]; oy = [p[1] for p in outer]
    margin = max(max(ox) - min(ox), max(oy) - min(oy)) * 0.05
    ax.set_xlim(min(ox) - margin, max(ox) + margin)
    ax.set_ylim(min(oy) - margin, max(oy) + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.text(0.02, 0.98, f'Cells: {len(cells_data)} | ● entry  ■ exit',
            transform=ax.transAxes, fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    plt.savefig(output_png, dpi=120)
    plt.close()
    print(f'  Rendered cells: {output_png}')


def render_connections(scenario_name, outer, holes, cells_data, connections_data, output_png):
    """Cell 间 route 控制点图：cell 边界 + 起止点 + 连接控制线。"""
    import matplotlib.cm as cm
    import numpy as np
    from matplotlib.patches import Polygon as MPolygon

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_title(f'{scenario_name} - Cell Route Waypoints', fontsize=12)

    # 外边界 + 孔洞
    poly = MPolygon(outer, fill=True, facecolor='lightyellow', edgecolor='black', linewidth=2)
    ax.add_patch(poly)
    for h in holes:
        hp = MPolygon(h, fill=True, facecolor='gray', edgecolor='darkred', linewidth=1.5, alpha=0.7)
        ax.add_patch(hp)

    # 每个 cell 边界浅色 + 起止点
    colors = cm.tab10(np.linspace(0, 1, max(len(cells_data), 1)))
    for ci, cell in enumerate(cells_data):
        boundary = cell.get('boundary', [])
        if len(boundary) < 3:
            continue
        color = colors[ci % len(colors)]
        cp = MPolygon(boundary, fill=True, facecolor=color, edgecolor='black',
                      linewidth=1, alpha=0.6)
        ax.add_patch(cp)
        cx = sum(p[0] for p in boundary) / len(boundary)
        cy = sum(p[1] for p in boundary) / len(boundary)
        ax.text(cx, cy, f'C{ci}', fontsize=8, ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6))
        if 'entry' in cell:
            ax.plot(cell['entry']['x'], cell['entry']['y'], 'o', color='green',
                    markersize=10, markeredgewidth=1.5, markeredgecolor='darkgreen', zorder=5)
        if 'exit' in cell:
            ax.plot(cell['exit']['x'], cell['exit']['y'], 's', color='red',
                    markersize=10, markeredgewidth=1.5, markeredgecolor='darkred', zorder=5)

    # Cell 间 route 控制线；最终可执行曲线以 coverage 图中的 path 为准。
    for conn in connections_data:
        path = conn.get('path', [])
        if len(path) >= 2:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, '--', color='#c8404a', lw=2, alpha=0.8)
            # 箭头（路径中点）
            mid = len(path) // 2
            ax.plot(px[mid], py[mid], '>', color='#c8404a', markersize=8, alpha=0.9)
        # 兼容旧格式 from/to
        elif 'from' in conn and 'to' in conn:
            fx, fy = conn['from']['x'], conn['from']['y']
            tx, ty = conn['to']['x'], conn['to']['y']
            ax.plot([fx, tx], [fy, ty], '--', color='#c8404a', lw=2, alpha=0.8)
        # 连接标注
        path = conn.get('path', [])
        if len(path) >= 2:
            mid = len(path) // 2
            ax.text(path[mid][0], path[mid][1],
                    f"C{conn['from_cell']}→C{conn['to_cell']}",
                    fontsize=7, ha='center', va='bottom', color='#c8404a',
                    bbox=dict(boxstyle='round,pad=0.1', facecolor='white', alpha=0.7))

    # 按多边形包围盒裁剪视图，避免大空白
    ox = [p[0] for p in outer]; oy = [p[1] for p in outer]
    margin = max(max(ox) - min(ox), max(oy) - min(oy)) * 0.05
    ax.set_xlim(min(ox) - margin, max(ox) + margin)
    ax.set_ylim(min(oy) - margin, max(oy) + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.text(0.02, 0.98, f'Cells: {len(cells_data)} | Route links: {len(connections_data)} | ○ entry  ■ exit',
            transform=ax.transAxes, fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    plt.savefig(output_png, dpi=120)
    plt.close()
    print(f'  Rendered connections: {output_png}')


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print('Usage: render_coverage.py <name> <polygon.yaml> <data.json> <output.png> [grid.json] [--mode cells|connections]')
        sys.exit(1)

    name = sys.argv[1]
    with open(sys.argv[2]) as f:
        data = yaml.safe_load(f)
    outer = [(p[0], p[1]) for p in data['polygon']]
    holes = [[(p[0], p[1]) for p in h] for h in data.get('holes', [])]

    with open(sys.argv[3]) as f:
        jd = json.load(f)
    path_pts = jd.get('path', [])
    component_paths = jd.get('component_paths', [])
    eval_r = jd.get('eval', {})
    if not eval_r and jd.get('component_evals'):
        component_evals = jd['component_evals']
        eval_r = {
            'coverage_rate': min(
                e.get('coverage_rate', 0) or 0 for e in component_evals),
            'single_score': min(
                e.get('single_score', 0) or 0 for e in component_evals),
        }
    cells_data = jd.get('cells', [])
    connections_data = jd.get('connections', [])

    grid_json = sys.argv[5] if len(sys.argv) > 5 and not sys.argv[5].startswith('--') else None
    if not grid_json and component_paths:
        component_grid_pattern = sys.argv[3].replace(
            '_data.json', '_grid_component_*.json')
        component_grids = sorted(glob.glob(component_grid_pattern))
        if component_grids:
            grid_json = component_grids
    mode = 'coverage'  # default
    for a in sys.argv:
        if a.startswith('--mode='):
            mode = a.split('=', 1)[1]
        elif a == '--mode' and sys.argv.index(a) + 1 < len(sys.argv):
            mode = sys.argv[sys.argv.index(a) + 1]

    if mode == 'cells':
        render_cells(name, outer, holes, cells_data, sys.argv[4])
    elif mode == 'connections':
        render_connections(name, outer, holes, cells_data, connections_data, sys.argv[4])
    else:
        render(name, outer, holes, path_pts, eval_r, sys.argv[4],
               grid_json, component_paths)
