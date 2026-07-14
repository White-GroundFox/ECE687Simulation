#!/usr/bin/env python3
"""
ECE 486 hockey controller for the RoboMaster EP, using the *approximate
linearization* (point-offset feedback linearization) method required by the
project, with the robot modeled as a UNICYCLE.

Model (unicycle):  xdot = v cos(theta), ydot = v sin(theta), thetadot = omega
Control point p a distance l in front of the robot (= stick tip):
    p     = [x + l cos(theta), y + l sin(theta)]
    pdot  = R(theta) L(l) [v, omega]^T ,   L(l) = diag(1, l)
=>  [v, omega]^T = L^-1(l) R^T(theta) pdot          (Eq. 3 in the PDF)

We design a simple proportional controller for pdot:  pdot = k (p_des - p),
then map back to (v, omega). ONLY v (linear.x) and omega (angular.z) are sent;
linear.y stays 0 (the unicycle model has no lateral input).

Tasks (state machine):
    T1 GO_TO_STICK : drive to known stick pick-up location
    T2 PICK_STICK  : close gripper on the stick (+ optional arm posture)
    T3 GO_TO_PUCK  : drive to a line-up point behind the puck
    T4 SHOOT       : drive the stick tip through the puck toward the goal

Object poses come from Vicon via vrpn_mocap (geometry_msgs/PoseStamped),
e.g. /vrpn_mocap/robot9/pose, /vrpn_mocap/stick1/pose, /vrpn_mocap/puck1/pose,
/vrpn_mocap/goal1/pose. Override the topic params to match the sim/lab names.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped

import robomaster_msgs.action


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lim):
    return max(-lim, min(lim, v))


class UnicycleHockey(Node):
    def __init__(self) -> None:
        super().__init__('unicycle_hockey')

        ns = self.declare_parameter('robot', 'robot9').value
        mocap = self.declare_parameter('mocap_ns', 'vrpn_mocap').value
        self.declare_parameter('cmd_vel_topic', f'/{ns}/cmd_vel')
        self.declare_parameter('robot_pose_topic', f'/{mocap}/{ns}/pose')
        self.declare_parameter('stick_pose_topic', f'/{mocap}/stick1/pose')
        self.declare_parameter('puck_pose_topic',  f'/{mocap}/puck1/pose')
        self.declare_parameter('goal_pose_topic',  f'/{mocap}/goal1/pose')

        # Geometry / gains
        self.l = self.declare_parameter('l', 0.20).value             # control-point offset (m)
        self.l_stick = self.declare_parameter('l_stick', 0.45).value # offset once stick is held
        self.k = self.declare_parameter('k', 1.0).value              # proportional gain on pdot
        self.v_max = self.declare_parameter('v_max', 0.5).value
        self.w_max = self.declare_parameter('w_max', 1.5).value

        # Strategy distances / tolerances (m)
        self.d_behind = self.declare_parameter('d_behind', 0.20).value
        self.d_through = self.declare_parameter('d_through', 0.25).value
        self.reach_tol = self.declare_parameter('reach_tol', 0.05).value
        self.goal_tol = self.declare_parameter('goal_tol', 0.15).value

        # Poses (np.array [x, y, theta] for robot; [x, y] for objects)
        self.robot = self.stick = self.puck = self.goal = None
        self.phase = 'GO_TO_STICK'
        self._busy = False  # set during the (async) gripper action

        self._sub('robot_pose_topic', self._on_robot)
        self._sub('stick_pose_topic', lambda m: setattr(self, 'stick', self._xy(m)))
        self._sub('puck_pose_topic',  lambda m: setattr(self, 'puck',  self._xy(m)))
        self._sub('goal_pose_topic',  lambda m: setattr(self, 'goal',  self._xy(m)))

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.gripper = ActionClient(
            self, robomaster_msgs.action.GripperControl, f'/{ns}/gripper')
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz
        self.get_logger().info('Unicycle hockey controller (approximate linearization) started.')

    # ---- small helpers ---------------------------------------------------
    def _sub(self, param, cb):
        self.create_subscription(PoseStamped, self.get_parameter(param).value, cb, 10)

    @staticmethod
    def _xy(m: PoseStamped) -> np.ndarray:
        return np.array([m.pose.position.x, m.pose.position.y])

    def _on_robot(self, m: PoseStamped) -> None:
        self.robot = np.array([m.pose.position.x, m.pose.position.y,
                               yaw_from_quat(m.pose.orientation)])

    def control_point(self) -> np.ndarray:
        x, y, th = self.robot
        return np.array([x + self.l * math.cos(th), y + self.l * math.sin(th)])

    # ---- APPROXIMATE LINEARIZATION: pdot -> (v, omega) -------------------
    def drive_point_to(self, p_des: np.ndarray) -> float:
        """Send (v, omega) to move the control point toward p_des. Returns the error norm."""
        x, y, th = self.robot
        p = self.control_point()
        e = p_des - p
        pdot = self.k * e                                  # proportional controller for pdot
        # [v; omega] = L^-1(l) R^T(theta) pdot
        c, s = math.cos(th), math.sin(th)
        v = c * pdot[0] + s * pdot[1]
        w = (-s * pdot[0] + c * pdot[1]) / self.l
        tw = Twist()
        tw.linear.x = clamp(float(v), self.v_max)
        tw.angular.z = clamp(float(w), self.w_max)
        self.cmd_pub.publish(tw)
        return float(np.linalg.norm(e))

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def unit_puck_to_goal(self) -> np.ndarray:
        u = self.goal - self.puck
        n = np.linalg.norm(u)
        return u / n if n > 1e-6 else np.array([1.0, 0.0])

    # ---- STATE MACHINE ---------------------------------------------------
    def step(self) -> None:
        if any(o is None for o in (self.robot, self.stick, self.puck, self.goal)):
            return  # wait for all mocap poses
        if self._busy:
            return  # a gripper action is running; hold position

        if self.phase == 'GO_TO_STICK':                      # T1
            if self.drive_point_to(self.stick) < self.reach_tol:
                self.stop()
                self.get_logger().info('Reached stick -> PICK_STICK')
                self.phase = 'PICK_STICK'

        elif self.phase == 'PICK_STICK':                     # T2
            self.close_gripper()                             # async; sets _busy

        elif self.phase == 'GO_TO_PUCK':                     # T3
            u = self.unit_puck_to_goal()
            lineup = self.puck - self.d_behind * u           # behind puck, on puck->goal line
            if self.drive_point_to(lineup) < self.reach_tol:
                self.stop()
                self.get_logger().info('Lined up behind puck -> SHOOT')
                self.phase = 'SHOOT'

        elif self.phase == 'SHOOT':                          # T4
            u = self.unit_puck_to_goal()
            through = self.puck + self.d_through * u          # drive stick tip through the puck
            self.drive_point_to(through)
            if np.linalg.norm(self.puck - self.goal) < self.goal_tol:
                self.stop()
                self.get_logger().info('GOAL! -> DONE')
                self.phase = 'DONE'

        elif self.phase == 'DONE':
            self.stop()

    # ---- T2 gripper action ----------------------------------------------
    def close_gripper(self) -> None:
        self._busy = True
        self.gripper.wait_for_server()
        goal = robomaster_msgs.action.GripperControl.Goal()
        goal.target_state = 2   # CLOSE
        goal.power = 0.7
        self.get_logger().info('Closing gripper on stick...')
        self.gripper.send_goal_async(goal).add_done_callback(self._grip_accepted)

    def _grip_accepted(self, future):
        future.result().get_result_async().add_done_callback(self._grip_done)

    def _grip_done(self, _):
        self.l = self.l_stick   # control point now at the stick tip
        self.get_logger().info(f'Stick grabbed; control offset l -> {self.l} m. -> GO_TO_PUCK')
        self.phase = 'GO_TO_PUCK'
        self._busy = False


def main() -> None:
    rclpy.init()
    node = UnicycleHockey()
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
