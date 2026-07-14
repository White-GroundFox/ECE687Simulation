#!/usr/bin/env python3
"""
FOCUSED TEST: pick up the stick (Tasks T1 + T2 only).  Single robot, no puck,
no obstacle avoidance -- the smallest thing that exercises navigate + grasp.

Sequence:
  OPEN     : open the gripper (and optionally set an arm posture/height)
  PREGRASP : drive the GRIPPER point to a point behind the stick, on the stick
             axis -> this makes the robot heading line up with the grasp axis
  GRASP    : drive forward along that axis until the gripper is at the stick
  CLOSE    : close the gripper on the stick
  DONE     : stop

Uses the same unicycle + approximate-linearization mapping as the project
controllers. Start SLOW (v_max small) and keep a hand on Ctrl-C: on exit the
node publishes a zero Twist to stop the robot.

Run (inside the container, robot + vrpn_mocap up):
  python3 /hockey/test_pickup.py --ros-args \
    -p robot:=robot9 -p stick_pose_topic:=/vrpn_mocap/stick1/pose \
    -p l_grip:=0.18 -p v_max:=0.2
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, Point

import robomaster_msgs.action


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PickupTest(Node):
    def __init__(self) -> None:
        super().__init__('pickup_test')
        ns = self.declare_parameter('robot', 'robot9').value
        mocap = self.declare_parameter('mocap_ns', 'vrpn_mocap').value
        self.declare_parameter('cmd_vel_topic', f'/{ns}/cmd_vel')
        self.declare_parameter('robot_pose_topic', f'/{mocap}/{ns}/pose')
        self.declare_parameter('stick_pose_topic', f'/{mocap}/stick1/pose')

        # geometry / gains  (MEASURE l_grip = robot center -> gripper, in meters)
        self.l = self.declare_parameter('l_grip', 0.18).value
        self.k = self.declare_parameter('k', 0.8).value
        self.v_max = self.declare_parameter('v_max', 0.2).value     # keep SLOW for first tests
        self.w_max = self.declare_parameter('w_max', 1.0).value
        self.d_pre = self.declare_parameter('d_pre', 0.25).value    # pre-grasp standoff (m)
        self.tol_pre = self.declare_parameter('tol_pre', 0.06).value
        self.tol_grasp = self.declare_parameter('tol_grasp', 0.03).value
        self.yaw_offset = self.declare_parameter('grasp_yaw_offset', 0.0).value  # tune to stick frame

        # optional arm posture (height to match the stick); off by default
        self.use_arm = self.declare_parameter('use_arm', False).value
        self.arm_x = self.declare_parameter('arm_x', 0.15).value
        self.arm_z = self.declare_parameter('arm_z', 0.05).value

        self.robot = None            # [x, y, theta]
        self.stick = None            # [x, y, yaw]
        self._busy = False
        self.phase = 'OPEN'

        self.create_subscription(PoseStamped, self.get_parameter('robot_pose_topic').value,
                                 self._on_robot, 10)
        self.create_subscription(PoseStamped, self.get_parameter('stick_pose_topic').value,
                                 self._on_stick, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.arm_pub = self.create_publisher(Point, f'/{ns}/target_arm_position', 1)
        self.gripper = ActionClient(self, robomaster_msgs.action.GripperControl, f'/{ns}/gripper')
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info('Pickup test started. Phase OPEN.')

    def _on_robot(self, m):
        self.robot = np.array([m.pose.position.x, m.pose.position.y,
                               yaw_from_quat(m.pose.orientation)])

    def _on_stick(self, m):
        self.stick = np.array([m.pose.position.x, m.pose.position.y,
                               yaw_from_quat(m.pose.orientation)])

    def gripper_point(self):
        x, y, th = self.robot
        return np.array([x + self.l * math.cos(th), y + self.l * math.sin(th)])

    def drive_to(self, p_des):
        x, y, th = self.robot
        p = self.gripper_point()
        u = self.k * (p_des - p)
        n = np.linalg.norm(u)
        if n > self.v_max:
            u *= self.v_max / n
        c, s = math.cos(th), math.sin(th)
        tw = Twist()
        tw.linear.x = float(c * u[0] + s * u[1])
        tw.angular.z = float(max(-self.w_max, min(self.w_max, (-s * u[0] + c * u[1]) / self.l)))
        self.cmd_pub.publish(tw)
        return float(np.linalg.norm(p_des - p))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def grasp_axis(self):
        """Unit vector along which the gripper approaches the stick."""
        yaw = self.stick[2] + self.yaw_offset
        return np.array([math.cos(yaw), math.sin(yaw)])

    def step(self):
        if self.robot is None or self.stick is None or self._busy:
            return
        stick_xy = self.stick[:2]
        axis = self.grasp_axis()

        if self.phase == 'OPEN':
            self._gripper(target=1)                       # OPEN
            return

        if self.phase == 'PREGRASP':
            pre = stick_xy - self.d_pre * axis            # behind the stick along its axis
            if self.drive_to(pre) < self.tol_pre:
                self.stop()
                self.get_logger().info('At pre-grasp -> GRASP')
                self.phase = 'GRASP'

        elif self.phase == 'GRASP':
            if self.drive_to(stick_xy) < self.tol_grasp:  # forward along the axis onto the stick
                self.stop()
                self.get_logger().info('Gripper at stick -> CLOSE')
                self.phase = 'CLOSE'

        elif self.phase == 'CLOSE':
            self._gripper(target=2)                       # CLOSE

        elif self.phase == 'DONE':
            self.stop()

    # --- gripper action; on completion advance the state machine ----------
    def _gripper(self, target):
        self._busy = True
        self.gripper.wait_for_server()
        g = robomaster_msgs.action.GripperControl.Goal()
        g.target_state, g.power = target, 0.7
        self.get_logger().info(f'Gripper -> {"OPEN" if target == 1 else "CLOSE"}')
        self.gripper.send_goal_async(g).add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(self._gripper_done))

    def _gripper_done(self, _):
        if self.phase == 'OPEN':
            if self.use_arm:
                self.arm_pub.publish(Point(x=float(self.arm_x), z=float(self.arm_z)))
                self.get_logger().info(f'Arm -> ({self.arm_x}, {self.arm_z})')
            self.phase = 'PREGRASP'
        elif self.phase == 'CLOSE':
            self.get_logger().info('Stick grasped. DONE.')
            self.phase = 'DONE'
        self._busy = False


def main():
    rclpy.init()
    node = PickupTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
