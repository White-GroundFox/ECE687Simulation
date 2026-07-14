#!/usr/bin/env python3
"""
Jacobian-based hockey controller for the DJI RoboMaster EP (robomaster_ros).

The robot holds a hockey stick in its gripper. We model the *stick tip* as the
task-space end effector and control its planar position (px, py) plus the robot
heading (theta) so the blade lines up with the puck->goal line and pushes the
puck into the goal.

Actuators (holonomic mecanum base, body frame):  q = [vx, vy, wz]  -> /cmd_vel
Task:                                            x = [px, py, theta]

Task Jacobian (see derivation):
    J = [[ cos -sin  -ry ],
         [ sin  cos   rx ],
         [  0    0     1 ]]            with (rx, ry) = R(theta) @ (Lx, Ly)
det(J) = 1, so it is always invertible (resolved-rate, no DLS needed).

PERCEPTION IS PLUGGABLE. This node only needs, in ONE common world/rink frame:
  - robot pose      (geometry_msgs/PoseStamped)  -> param 'robot_pose_topic'
  - puck position   (geometry_msgs/PointStamped) -> param 'puck_topic'
  - goal position   (fixed)                      -> params 'goal_x', 'goal_y'
Feed those from whatever you have (overhead camera + markers, motion capture,
onboard vision...). Swap the message types/topics in _setup_perception() to match.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped


def yaw_from_quat(q) -> float:
    """Yaw (rotation about z) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class HockeyJacobianController(Node):
    def __init__(self) -> None:
        super().__init__('hockey_jacobian_controller')

        # --- Robot namespace / topics --------------------------------------
        ns = self.declare_parameter('robot', 'robot9').value
        self.declare_parameter('cmd_vel_topic', f'/{ns}/cmd_vel')
        self.declare_parameter('robot_pose_topic', f'/{ns}/pose')   # PoseStamped, world frame
        self.declare_parameter('puck_topic', '/puck/pose')          # PointStamped, world frame

        # --- Geometry: stick tip offset in the BODY frame (METERS) ----------
        # Lx = distance from chassis center to stick tip, forward.
        # MEASURE THIS on the real robot (chassis center -> blade contact point).
        self.Lx = self.declare_parameter('stick_Lx', 0.35).value
        self.Ly = self.declare_parameter('stick_Ly', 0.0).value

        # --- Goal location in the world frame (METERS) ----------------------
        self.goal = np.array([
            self.declare_parameter('goal_x', 2.0).value,
            self.declare_parameter('goal_y', 0.0).value,
        ])

        # --- Control gains and limits ---------------------------------------
        self.Kp = np.diag([
            self.declare_parameter('kx', 1.2).value,
            self.declare_parameter('ky', 1.2).value,
            self.declare_parameter('ktheta', 2.0).value,
        ])
        self.v_max = self.declare_parameter('v_max', 0.6).value      # m/s
        self.w_max = self.declare_parameter('w_max', 1.5).value      # rad/s

        # --- Strategy distances (METERS) ------------------------------------
        self.d_behind = self.declare_parameter('d_behind', 0.15).value   # line-up point behind puck
        self.d_through = self.declare_parameter('d_through', 0.20).value  # follow-through past puck
        self.align_tol = self.declare_parameter('align_tol', 0.04).value # m, "tip is in position"
        self.heading_tol = self.declare_parameter('heading_tol', 0.15).value  # rad
        self.goal_tol = self.declare_parameter('goal_tol', 0.15).value   # m, puck in goal

        # --- State ----------------------------------------------------------
        self.robot = None    # np.array([x, y, theta])
        self.puck = None     # np.array([x, y])
        self.phase = 'ALIGN'

        self._setup_perception()
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.timer = self.create_timer(0.05, self.control_step)  # 20 Hz
        self.get_logger().info('Hockey Jacobian controller started.')

    # ----------------------------------------------------------------------
    # PERCEPTION  (adapt these to your actual sensing setup)
    # ----------------------------------------------------------------------
    def _setup_perception(self) -> None:
        self.create_subscription(
            PoseStamped, self.get_parameter('robot_pose_topic').value, self._on_robot, 10)
        self.create_subscription(
            PointStamped, self.get_parameter('puck_topic').value, self._on_puck, 10)

    def _on_robot(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.robot = np.array([p.x, p.y, yaw_from_quat(msg.pose.orientation)])

    def _on_puck(self, msg: PointStamped) -> None:
        self.puck = np.array([msg.point.x, msg.point.y])

    # ----------------------------------------------------------------------
    # KINEMATICS
    # ----------------------------------------------------------------------
    def stick_tip(self) -> np.ndarray:
        """Current stick-tip position in the world frame."""
        x, y, th = self.robot
        c, s = math.cos(th), math.sin(th)
        rx = c * self.Lx - s * self.Ly
        ry = s * self.Lx + c * self.Ly
        return np.array([x + rx, y + ry])

    def jacobian(self, th: float) -> np.ndarray:
        c, s = math.cos(th), math.sin(th)
        rx = c * self.Lx - s * self.Ly
        ry = s * self.Lx + c * self.Ly
        return np.array([
            [c, -s, -ry],
            [s,  c,  rx],
            [0,  0,  1.0],
        ])

    # ----------------------------------------------------------------------
    # STRATEGY: choose the desired task-space target [px, py, theta]
    # ----------------------------------------------------------------------
    def desired_task(self) -> np.ndarray:
        puck = self.puck
        to_goal = self.goal - puck
        dist = np.linalg.norm(to_goal)
        u = to_goal / dist if dist > 1e-6 else np.array([1.0, 0.0])  # unit puck->goal
        theta_des = math.atan2(u[1], u[0])  # face the goal direction

        if self.phase == 'ALIGN':
            target = puck - self.d_behind * u        # sit behind the puck, on the line
        elif self.phase == 'STRIKE':
            target = puck + self.d_through * u        # drive through the puck toward goal
        else:  # RECOVER
            target = puck - (self.d_behind + 0.25) * u
        return np.array([target[0], target[1], theta_des])

    def update_phase(self, tip: np.ndarray, x_des: np.ndarray) -> None:
        puck_in_goal = np.linalg.norm(self.puck - self.goal) < self.goal_tol
        pos_err = np.linalg.norm(x_des[:2] - tip)
        head_err = abs(wrap(x_des[2] - self.robot[2]))

        if puck_in_goal:
            self.phase = 'RECOVER'
        elif self.phase == 'ALIGN' and pos_err < self.align_tol and head_err < self.heading_tol:
            self.get_logger().info('Aligned -> STRIKE')
            self.phase = 'STRIKE'
        elif self.phase == 'STRIKE' and pos_err < self.align_tol:
            self.get_logger().info('Strike follow-through done -> ALIGN')
            self.phase = 'ALIGN'

    # ----------------------------------------------------------------------
    # CONTROL LOOP
    # ----------------------------------------------------------------------
    def control_step(self) -> None:
        if self.robot is None or self.puck is None:
            return  # waiting for perception

        tip = self.stick_tip()
        x_des = self.desired_task()
        self.update_phase(tip, x_des)
        x_des = self.desired_task()  # phase may have changed

        # Task-space error (wrap the heading component)
        err = np.array([
            x_des[0] - tip[0],
            x_des[1] - tip[1],
            wrap(x_des[2] - self.robot[2]),
        ])

        # Resolved-rate control: q_dot = J^-1 * Kp * err   (body-frame vx, vy, wz)
        J = self.jacobian(self.robot[2])
        q_dot = np.linalg.solve(J, self.Kp @ err)

        twist = Twist()
        twist.linear.x = clamp(float(q_dot[0]), -self.v_max, self.v_max)
        twist.linear.y = clamp(float(q_dot[1]), -self.v_max, self.v_max)
        twist.angular.z = clamp(float(q_dot[2]), -self.w_max, self.w_max)
        self.cmd_pub.publish(twist)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = HockeyJacobianController()
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
