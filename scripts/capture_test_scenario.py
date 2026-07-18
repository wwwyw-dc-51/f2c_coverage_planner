#!/usr/bin/env python3
"""
单场景测试与数据采集工具
1. 启动 planner 子进程
2. 发布多边形+孔洞
3. 订阅路径 topic，保存数据
4. 等待评估完成，提取结果
5. 输出 JSON
"""
import sys, os, time, yaml, json, subprocess, re

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path

HOLE_SEPARATOR = 1e10

class TestCapture(Node):
    def __init__(self, yaml_path, scenario_name, ws_path):
        super().__init__('test_capture_' + scenario_name)
        self.scenario = scenario_name
        self.ws = ws_path
        self.yaml_path = yaml_path

        self.path_pts = []
        self.swaths_data = []
        self.path_received = False
        self.eval_text = ''

        self.path_sub = self.create_subscription(Path, '/planned2_path_1', self.on_path, 10)
        self.poly_pub = self.create_publisher(Polygon, '/input_polygon_1', 10)
        self.holes_pub = self.create_publisher(Polygon, '/input_polygon_1_holes', 10)

    def on_path(self, msg):
        if len(msg.poses) > 0 and not self.path_received:
            for pose in msg.poses:
                self.path_pts.append({
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y
                })
            self.path_received = True

    def publish_polygon(self):
        with open(self.yaml_path) as f:
            area = yaml.safe_load(f)

        polygon = area['polygon']
        holes = area.get('holes', [])

        poly_msg = Polygon()
        for p in polygon:
            poly_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

        holes_msg = Polygon()
        for hi, hole in enumerate(holes):
            if hi > 0:
                holes_msg.points.append(Point32(x=HOLE_SEPARATOR, y=0.0, z=0.0))
            for p in hole:
                holes_msg.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

        for attempt in range(5):
            self.holes_pub.publish(holes_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
            self.poly_pub.publish(poly_msg)

            waited = 0.0
            while waited < 15.0:
                rclpy.spin_once(self, timeout_sec=0.5)
                waited += 0.5
                if self.path_received:
                    print(f'  Plan confirmed after attempt {attempt+1}')
                    return
            print(f'  Attempt {attempt+1}: no plan, retrying...')
        print('  WARNING: no plan received')

    def run_test(self):
        result_dir = f'{self.ws}/test_results'
        os.makedirs(result_dir, exist_ok=True)
        log_file = f'{result_dir}/{self.scenario}_planner.log'

        env = os.environ.copy()
        ld_path = env.get('LD_LIBRARY_PATH', '')
        extra_paths = [
            f'{self.ws}/install/lib',
            f'{self.ws}/src/Fields2Cover/build/_deps/steering_functions-build',
            f'{self.ws}/src/Fields2Cover/third_party/ortools-src/lib',
        ]
        env['LD_LIBRARY_PATH'] = ':'.join(extra_paths + ([ld_path] if ld_path else []))

        planner_cmd = [
            'ros2', 'run', 'yingshi_robot', 'polygon_planner_node', '--ros-args',
            '-p', 'robot_width:=0.95',
            '-p', 'coverage_width:=0.45',
            '-p', 'mid_hl_width_ratio:=0.20',
            '-p', 'no_hl_width_ratio:=0.0',
            '-p', 'min_hole_area:=1.0',
            '-p', 'decomposition_angle:=0.0',
            '-p', 'swath_endpoint_shrink_distance:=0.03',
            '-p', 'min_swath_length:=0.5',
            '-p', 'max_diff_curv:=0.3',
            '-p', 'path_resolution:=0.1',
            '-p', 'boundary_type:=closed',
            '-p', 'use_optimized_planner:=true',
            '-p', 'swath_angle_optimization:=true',
            '-p', 'decomposition_angle_optimization:=false',
            '-p', 'decomposition_enabled:=true',
            '-p', 'use_sweep_decomp:=true',
            '-p', 'filter_tiny_cells:=true',
            '-p', 'path_simplify_enabled:=true',
            '-p', 'path_simplify_tolerance:=0.05',
            '-p', 'path_simplify_turn_threshold:=0.15',
            '-p', 'turn_planner_type:=direct',
            '-p', 'swath_overlap_ratio:=0.03',
            '-p', 'eval_enable_report:=true',
            '-p', 'eval_use_grid_method:=true',
            '-p', 'eval_grid_resolution:=0.1',
            '-p', 'eval_coverage_threshold:=0.99',
        ]

        with open(log_file, 'w') as lf:
            planner = subprocess.Popen(planner_cmd, env=env, stdout=lf, stderr=subprocess.STDOUT)

        time.sleep(2.0)

        self.publish_polygon()

        print('  Waiting for evaluation...')
        for i in range(90):
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            if os.path.exists(log_file):
                with open(log_file) as f:
                    if '综合得分' in f.read():
                        self.eval_text = open(log_file).read()
                        print(f'  Evaluation complete after {i+1}s')
                        break

        planner.terminate()
        try:
            planner.wait(timeout=5)
        except:
            planner.kill()

        return log_file

    def parse_eval(self):
        result = {}
        if not self.eval_text:
            return result

        patterns = {
            'coverage_rate': r'覆盖率[:\s]*([\d.]+)%',
            'single_score': r'综合得分[:\s]*([\d.]+)',
            'uncovered_area': r'未覆盖面积[:\s]*([\d.]+)',
            'total_distance': r'路径总长[:\s]*([\d.]+)',
            'work_ratio': r'有效工作比[:\s]*([\d.]+)%',
            'turn_count': r'转弯次数[:\s]*(\d+)',
            'overlap_rate': r'重叠率[:\s]*([\d.]+)%',
            'planning_time_ms': r'规划耗时[:\s]*([\d.]+)\s*ms',
            'coverage_method': r'覆盖率方法[:\s]*(\w+)',
            'net_area': r'目标净面积[:\s]*([\d.]+)',
        }

        for key, pat in patterns.items():
            m = re.search(pat, self.eval_text)
            if m:
                val = m.group(1)
                if key in ('turn_count',):
                    result[key] = int(val)
                elif key == 'coverage_method':
                    result[key] = val
                elif key == 'coverage_rate':
                    result[key] = float(val) / 100.0
                else:
                    result[key] = float(val)

        return result

    def extract_swaths_from_log(self, log_file):
        swaths = []
        if not os.path.exists(log_file):
            return swaths
        with open(log_file) as f:
            for line in f:
                m = re.search(r'Swath.*start=\(([\d.]+),\s*([\d.]+)\).*end=\(([\d.]+),\s*([\d.]+)\)', line)
                if m:
                    swaths.append({
                        'points': [
                            {'x': float(m.group(1)), 'y': float(m.group(2))},
                            {'x': float(m.group(3)), 'y': float(m.group(4))}
                        ]
                    })
        return swaths


def main():
    if len(sys.argv) < 3:
        print('Usage: test_and_capture.py <scenario_name> <polygon.yaml>')
        sys.exit(1)

    scenario = sys.argv[1]
    yaml_path = sys.argv[2]
    ws = os.path.expanduser('~/f2c_coverage_planner')

    rclpy.init()

    try:
        capture = TestCapture(yaml_path, scenario, ws)
        log_file = capture.run_test()

        eval_result = capture.parse_eval()

        if not capture.swaths_data:
            capture.swaths_data = capture.extract_swaths_from_log(log_file)

        output = {
            'scenario': scenario,
            'path': capture.path_pts,
            'swaths': capture.swaths_data,
            'eval': eval_result,
            'log_file': log_file
        }

        result_dir = f'{ws}/test_results'
        json_path = f'{result_dir}/{scenario}_data.json'
        with open(json_path, 'w') as f:
            json.dump(output, f, indent=2, ensure_ascii=False)

        print(f'  Data saved: {json_path}')
        cov = eval_result.get('coverage_rate', 0) * 100
        score = eval_result.get('single_score', 0)
        print(f'  Coverage: {cov:.2f}% | Score: {score:.1f}')

        capture.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
