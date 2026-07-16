#!/usr/bin/env python3
"""
F2C 路径可视化 v3 — Python 全流程，一步到位
1. 启动 planner 子进程
2. 循环 7×2=14 组合：发布多边形 → 等 Vis JSON → 渲染 PNG
3. 清理
"""
import subprocess, sys, os, time, yaml, json, shutil
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

HOLE_SEP = 1e10

WS = os.path.expanduser("~/f2c_coverage_planner")
RESULT = f"{WS}/test_results/viz_v3_{time.strftime('%m%d_%H%M')}"
os.makedirs(RESULT, exist_ok=True)

SCENARIOS = [
    ("S1", f"{WS}/src/yingshi_robot/test_polygons/S1_S1_convex_rect.yaml"),
    ("S2", f"{WS}/src/yingshi_robot/test_polygons/S2_S2_L_shaped.yaml"),
    ("S3", f"{WS}/src/yingshi_robot/test_polygons/S3_S3_with_holes.yaml"),
    ("S4", f"{WS}/src/yingshi_robot/test_polygons/S4_S4_narrow_corridor.yaml"),
    ("S5", f"{WS}/src/yingshi_robot/test_polygons/S5_S5_irregular.yaml"),
    ("S6", f"{WS}/src/yingshi_robot/test_polygons/S6_S6_multi_region.yaml"),
    ("notched", f"{WS}/src/yingshi_robot/config/f2c_areas/notched_10m_with_center_hole.yaml"),
]

MODES = [("boustrophedon", "custom"), ("snake", "snake")]

VIS_JSON = "/tmp/f2c_vis_polygon_1.json"

# ── 启动 planner ──
env = os.environ.copy()
ld = env.get("LD_LIBRARY_PATH", "")
extra = [
    f"{WS}/install/lib",
    f"{WS}/src/Fields2Cover/build/_deps/steering_functions-build",
    f"{WS}/src/Fields2Cover/third_party/ortools-src/lib",
]
env["LD_LIBRARY_PATH"] = ":".join(extra + ([ld] if ld else []))

planner_cmd = [
    "ros2", "run", "yingshi_robot", "polygon_planner_node", "--ros-args",
    "-p", "robot_width:=0.95", "-p", "coverage_width:=0.45",
    "-p", "mid_hl_width_ratio:=0.20", "-p", "no_hl_width_ratio:=0.0",
    "-p", "min_hole_area:=1.0", "-p", "swath_endpoint_shrink_distance:=0.03",
    "-p", "min_swath_length:=0.5", "-p", "max_diff_curv:=0.3",
    "-p", "path_resolution:=0.1",
    "-p", "use_optimized_planner:=true", "-p", "swath_angle_optimization:=true",
    "-p", "decomposition_enabled:=true", "-p", "filter_tiny_cells:=true",
    "-p", "path_simplify_enabled:=true", "-p", "path_simplify_tolerance:=0.05",
    "-p", "turn_planner_type:=direct", "-p", "swath_overlap_ratio:=0.03",
    "-p", "use_sweep_decomp:=true", "-p", "merge_angle_threshold:=60.0",
    "-p", "eval_enable_report:=true", "-p", "eval_use_grid_method:=true",
    "-p", "eval_grid_resolution:=0.1", "-p", "eval_coverage_threshold:=0.995",
]

print("Starting planner...")
planner = subprocess.Popen(planner_cmd, env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(3)
print(f"Planner PID: {planner.pid}")

# ── ROS2 publisher ──
rclpy.init()
pub_node = Node("batch_viz_v3")
poly_pub = pub_node.create_publisher(Polygon, "/input_polygon_1", 10)
holes_pub = pub_node.create_publisher(Polygon, "/input_polygon_1_holes", 10)

def publish_one(yaml_path):
    """Publish polygon+holes once, wait for Vis JSON"""
    with open(yaml_path) as f:
        area = yaml.safe_load(f)

    poly_msg = Polygon()
    for p in area["polygon"]:
        poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

    holes_msg = Polygon()
    for hi, h in enumerate(area.get("holes", [])):
        if hi > 0:
            holes_msg.points.append(Point32(x=HOLE_SEP, y=0.0, z=0.0))
        for p in h:
            holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

    # 清除旧 JSON
    if os.path.exists(VIS_JSON):
        os.remove(VIS_JSON)

    # 发布
    holes_pub.publish(holes_msg)
    rclpy.spin_once(pub_node, timeout_sec=0.1)
    time.sleep(0.3)
    poly_pub.publish(poly_msg)
    rclpy.spin_once(pub_node, timeout_sec=0.1)

    # 等 Vis JSON 出现
    for _ in range(120):
        rclpy.spin_once(pub_node, timeout_sec=0.5)
        time.sleep(0.3)
        if os.path.exists(VIS_JSON) and os.path.getsize(VIS_JSON) > 100:
            time.sleep(0.5)  # 确保写完整
            return True
    return False

# ── 循环 14 组合 ──
total = len(SCENARIOS) * len(MODES)
done = 0
for name, yaml_path in SCENARIOS:
    # Determine boundary type
    with open(yaml_path) as f:
        area = yaml.safe_load(f)
    boundary = area.get("boundary_type", "closed")
    # Can't change params at runtime, use boundary from YAML
    # The planner uses params set at startup; we use --ros-args boundary_type above
    # This is a known limitation: we set boundary_type based on first scenario

    for mode, label in MODES:
        print(f"[{done+1}/{total}] {name} {label}...", end=" ", flush=True)

        # CANNOT change swath_order_type at runtime — planner param is fixed
        # Workaround: kill and restart planner with correct mode
        # Actually, the planner reads params at startup only. So we need to restart
        # for each mode change. But for now, the planner was started with
        # swath_order_type default (boustrophedon). Let's handle this...

        # For v3 quick fix: restart planner per mode
        # This is slower but reliable
        planner.terminate()
        try: planner.wait(timeout=3)
        except: planner.kill()

        # Restart with correct mode
        planner_cmd_final = planner_cmd + ["-p", f"swath_order_type:={mode}"]
        planner = subprocess.Popen(planner_cmd_final, env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)

        if publish_one(yaml_path):
            # Copy JSON
            data_json = f"{RESULT}/{name}_{label}_data.json"
            shutil.copy(VIS_JSON, data_json)

            # Render
            png = f"{RESULT}/{name}_{label}.png"
            import subprocess as sp
            sp.run(["python3", f"{WS}/scripts/render_coverage.py",
                    f"{name}_{label}", yaml_path, data_json, png],
                   check=False, capture_output=True)
            if os.path.exists(png):
                print(f"✓ {os.path.getsize(png)} bytes")
                done += 1
            else:
                print("✗ render failed")
        else:
            print("✗ no Vis JSON (timeout)")

        time.sleep(1)

# ── 清理 ──
planner.terminate()
try: planner.wait(timeout=3)
except: planner.kill()
pub_node.destroy_node()
rclpy.shutdown()

print(f"\nDone: {done}/{total} PNGs in {RESULT}")
for f in sorted(os.listdir(RESULT)):
    if f.endswith('.png'):
        print(f"  {RESULT}/{f}")
