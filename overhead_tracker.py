#!/usr/bin/env python3
"""
Overhead-camera tracker for the hockey setup (perception layer).

Pipeline:  camera frame --(HSV color blobs)--> pixel centroids
           --(homography H)--> rink/world meters --> ROS 2 topics

Markers (stick colored dots on top of things, distinct non-overlapping colors):
  - ROBOT FRONT  : one color  -> heading = atan2(front - back)
  - ROBOT BACK   : another    -> robot center = midpoint(front, back)
  - PUCK         : a third color
Avoid RED (it wraps around the HSV hue circle); orange/green/blue/yellow are easy.

Publishes (matching hockey_jacobian_control.py defaults):
  - robot pose : geometry_msgs/PoseStamped  on 'robot_pose_topic'
  - puck       : geometry_msgs/PointStamped on 'puck_topic'

Requires a homography file from calibrate_homography.py (default homography.npy).
"""

import math
import os

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped


def find_blob(hsv, lo, hi, min_area=80):
    """Return (u, v) pixel centroid of the largest blob in the HSV range, or None."""
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    m = cv2.moments(c)
    return np.array([m['m10'] / m['m00'], m['m01'] / m['m00']])


def pix_to_world(H, uv):
    """Apply the 3x3 homography to a pixel point -> (X, Y) in meters."""
    p = H @ np.array([uv[0], uv[1], 1.0])
    return np.array([p[0] / p[2], p[1] / p[2]])


class OverheadTracker(Node):
    def __init__(self) -> None:
        super().__init__('overhead_tracker')

        cam = self.declare_parameter('camera', '0').value          # index "0" or RTSP/URL string
        self.declare_parameter('robot_pose_topic', '/robot9/pose')
        self.declare_parameter('puck_topic', '/puck/pose')
        self.frame_id = self.declare_parameter('frame_id', 'rink').value
        hpath = self.declare_parameter('homography', 'homography.npy').value
        self.show = self.declare_parameter('show', True).value      # debug window (needs WSLg/X)

        if not os.path.exists(hpath):
            raise FileNotFoundError(
                f'Homography {hpath} not found. Run calibrate_homography.py first.')
        self.H = np.load(hpath)

        # HSV color ranges  [H(0-179), S(0-255), V(0-255)] -- TUNE with calibrate step.
        def rng(name, lo, hi):
            lo = self.declare_parameter(f'{name}_lo', lo).value
            hi = self.declare_parameter(f'{name}_hi', hi).value
            return list(lo), list(hi)
        self.front = rng('front', [10, 120, 120], [25, 255, 255])   # orange
        self.back = rng('back', [100, 120, 80], [130, 255, 255])    # blue
        self.puck = rng('puck', [40, 80, 80], [80, 255, 255])       # green

        src = int(cam) if cam.isdigit() else cam
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera source: {cam}')

        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('robot_pose_topic').value, 10)
        self.puck_pub = self.create_publisher(
            PointStamped, self.get_parameter('puck_topic').value, 10)
        self.timer = self.create_timer(1 / 30.0, self.tick)         # 30 Hz
        self.get_logger().info('Overhead tracker started.')

    def tick(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No camera frame', throttle_duration_sec=2.0)
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        now = self.get_clock().now().to_msg()

        f = find_blob(hsv, *self.front)
        b = find_blob(hsv, *self.back)
        p = find_blob(hsv, *self.puck)

        if f is not None and b is not None:
            fw, bw = pix_to_world(self.H, f), pix_to_world(self.H, b)
            center = 0.5 * (fw + bw)
            yaw = math.atan2(fw[1] - bw[1], fw[0] - bw[0])
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.pose.position.x, msg.pose.position.y = float(center[0]), float(center[1])
            msg.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.orientation.w = math.cos(yaw / 2)
            self.pose_pub.publish(msg)

        if p is not None:
            pw = pix_to_world(self.H, p)
            msg = PointStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.point.x, msg.point.y = float(pw[0]), float(pw[1])
            self.puck_pub.publish(msg)

        if self.show:
            for blob, col in ((f, (0, 165, 255)), (b, (255, 0, 0)), (p, (0, 255, 0))):
                if blob is not None:
                    cv2.circle(frame, (int(blob[0]), int(blob[1])), 8, col, 2)
            cv2.imshow('overhead', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OverheadTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
