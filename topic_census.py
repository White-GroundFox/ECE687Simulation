#!/usr/bin/env python3
"""Long-window topic discovery: like `ros2 topic list --no-daemon` but waits
WAIT_S seconds for DDS discovery instead of ~1 s. Usage:
    python3 /hockey/topic_census.py [seconds]
"""
import sys
import time

import rclpy

wait_s = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

rclpy.init()
node = rclpy.create_node('topic_census')
t0 = time.time()
while time.time() - t0 < wait_s:
    rclpy.spin_once(node, timeout_sec=0.25)

names = sorted(t for t, _ in node.get_topic_names_and_types())
for t in names:
    print(t)
print(f'--- {len(names)} topics after {wait_s:.0f}s window ---', file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
