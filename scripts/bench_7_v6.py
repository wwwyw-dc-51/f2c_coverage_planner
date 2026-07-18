#!/usr/bin/env python3
"""F2C 7场景基准测试 v6 — custom 算法，每场景重启planner"""
import subprocess, sys, os, time, yaml, json, shutil

WS = os.path.expanduser("~/f2c_coverage_planner")
RESULT = f"{WS}/test_results/bench_v6_{time.strftime('%m%d_%H%M')}"
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

VIS_JSON = "/tmp/f2c_vis_polygon_1.json"
RENDER_SCRIPT = f"{WS}/scripts/render_coverage.py"
HOLE_SEP = 1e10

# ── 环境变量 ──
env = os.environ.copy()
ld = env.get("LD_LIBRARY_PATH", "")
extra = [
    f"{WS}/install/lib",
    f"{WS}/src/Fields2Cover/build/_deps/steering_functions-build",
    f"{WS}/src/Fields2Cover/third_party/ortools-src/lib",
]
env["LD_LIBRARY_PATH"] = ":".join(extra + ([ld] if ld else []))

PLANNER_CMD = [
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
    "-p", "swath_order_type:=boustrophedon",
    "-p", "eval_enable_report:=true", "-p", "eval_use_grid_method:=true",
    "-p", "eval_grid_resolution:=0.1", "-p", "eval_coverage_threshold:=0.99",
]

def start_planner():
    """启动 planner 子进程"""
    p = subprocess.Popen(PLANNER_CMD, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)
    return p

def stop_planner(p):
    """停止 planner"""
    try:
        p.terminate()
        p.wait(timeout=3)
    except:
        p.kill()

# ── ROS2 publisher ──
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

rclpy.init()
pub_node = Node("bench_v6")
poly_pub = pub_node.create_publisher(Polygon, "/input_polygon_1", 10)
holes_pub = pub_node.create_publisher(Polygon, "/input_polygon_1_holes", 10)

def publish_one(yaml_path, timeout_sec=120):
    """发布一个场景的多边形和孔洞"""
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

    # Clear old JSON
    if os.path.exists(VIS_JSON):
        os.remove(VIS_JSON)

    # Publish holes first, wait, then polygon, then holes again
    holes_pub.publish(holes_msg)
    rclpy.spin_once(pub_node, timeout_sec=0.2)
    time.sleep(0.4)
    poly_pub.publish(poly_msg)
    rclpy.spin_once(pub_node, timeout_sec=0.2)
    time.sleep(0.4)
    holes_pub.publish(holes_msg)
    rclpy.spin_once(pub_node, timeout_sec=0.2)

    # Wait for JSON
    for _ in range(timeout_sec * 2):
        rclpy.spin_once(pub_node, timeout_sec=0.5)
        time.sleep(0.3)
        if os.path.exists(VIS_JSON) and os.path.getsize(VIS_JSON) > 500:
            time.sleep(0.5)
            return True
    return False

# ── Run 7 scenarios ──
results = []
for i, (name, yaml_path) in enumerate(SCENARIOS):
    print(f"[{i+1}/7] {name}...", end=" ", flush=True)

    # Start planner fresh for each scenario
    planner = start_planner()

    if publish_one(yaml_path):
        with open(VIS_JSON) as f:
            data = json.load(f)
        coverage = data["eval"]["coverage_rate"]
        score = data["eval"]["single_score"]
        n_pts = len(data["path"])

        # Copy JSON and render PNG
        json_out = f"{RESULT}/{name}_custom_data.json"
        shutil.copy(VIS_JSON, json_out)
        png_out = f"{RESULT}/{name}_custom.png"
        subprocess.run(["python3", RENDER_SCRIPT, f"{name}_custom",
                        yaml_path, VIS_JSON, png_out],
                       check=False, capture_output=True)

        results.append((name, coverage, score, n_pts))
        print(f"cov={coverage*100:.1f}% score={score:.1f} pts={n_pts}")
    else:
        print("TIMEOUT")
        results.append((name, 0.0, 0.0, 0))

    stop_planner(planner)
    time.sleep(1)

# ── Cleanup ──
pub_node.destroy_node()
rclpy.shutdown()

# ── Summary ──
print("\n" + "="*60)
print("  F2C v6 7-Scenario Benchmark Results")
print("="*60)
print(f"{'Scenario':<12} {'Coverage':>10} {'Score':>8} {'Points':>8}")
print("-"*42)
for name, cov, score, n_pts in results:
    print(f"{name:<12} {cov*100:>9.1f}% {score:>7.1f} {n_pts:>8}")
print("-"*42)
if results:
    avg_cov = sum(r[1] for r in results) / len(results)
    avg_score = sum(r[2] for r in results) / len(results)
    print(f"{'AVERAGE':<12} {avg_cov*100:>9.1f}% {avg_score:>7.1f}")
print(f"\nResults saved to: {RESULT}")
for f in sorted(os.listdir(RESULT)):
    if f.endswith('.png'):
        print(f"  {RESULT}/{f}")
