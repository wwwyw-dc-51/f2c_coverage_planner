#!/usr/bin/env python3
"""发送测试多边形到 polygon_planner_node 触发规划"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon

def main():
    if len(sys.argv) < 2:
        print("Usage: send_test_polygon.py <yaml_file>")
        sys.exit(1)

    rclpy.init()
    node = Node('test_sender')

    pub_poly = node.create_publisher(Polygon, '/input_polygon_1', 10)
    pub_holes = node.create_publisher(Polygon, '/input_polygon_1_holes', 10)

    import yaml
    with open(sys.argv[1]) as f:
        area = yaml.safe_load(f)

    pm = Polygon()
    for p in area['polygon']:
        pm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

    hm = Polygon()
    for hi, h in enumerate(area.get('holes', [])):
        if hi > 0:
            hm.points.append(Point32(x=1e10, y=0.0, z=0.0))
        for p in h:
            hm.points.append(Point32(x=float(p[0]), y=float(p[1]), z=0.0))

    # 先发几次孔洞，再发多边形
    for _ in range(3):
        pub_holes.publish(hm)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.2)
        pub_poly.publish(pm)
        time.sleep(2.0)
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"Polygon published: {len(area['polygon'])} vertices, "
          f"{len(area.get('holes', []))} holes")
    print("Waiting for planner to finish (45s)...")

    for _ in range(90):
        rclpy.spin_once(node, timeout_sec=0.5)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    print("Done.")

if __name__ == '__main__':
    main()
