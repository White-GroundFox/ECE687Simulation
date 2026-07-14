#!/usr/bin/env python3
"""
ECE 687 hockey controller: unicycle + approximate linearization, with
CLF-CBF-QP-based SAFE navigation (collision avoidance) and cooperative
pass/shoot roles.

Control stack (per robot):
  1. Model = unicycle. Control point p at offset l (= stick tip).
     approx. linearization:  [v; omega] = L^-1(l) R^T(theta) u   (u = pdot)
  2. Nominal controller (CLF):   u_nom = -k (p - p_des)
  3. Safety filter (CBF-QP):     u* = argmin ||u - u_nom||^2
                                 s.t.  grad h_i . u >= -alpha h_i   for each obstacle i
                                       |u| <= u_max
     h_i(p) = ||p - o_i||^2 - R_safe^2   (o_i = other robots + static obstacles)
  4. Map u* -> (v, omega) and publish (linear.y stays 0).

Roles (param 'role'):
  'passer'  : pick stick -> go to puck -> push puck toward the teammate receive point
  'shooter' : pick stick -> wait until puck is near -> go to puck -> shoot into goal
  'solo'    : pick stick -> go to puck -> shoot into goal   (single-robot test of the stack)

Poses come from vrpn_mocap (geometry_msgs/PoseStamped). Obstacles = the mocap
topics listed in 'obstacle_topics' (other robots and any static obstacle markers).
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped

import robomaster_msgs.action

try:
    from qpsolvers import solve_qp
    _HAVE_QP = True
except Exception:
    _HAVE_QP = False


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ECE687Hockey(Node):
    def __init__(self) -> None:
        super().__init__('ece687_hockey')

        ns = self.declare_parameter('robot', 'robot9').value
        mocap = self.declare_parameter('mocap_ns', 'vrpn_mocap').value
        self.role = self.declare_parameter('role', 'solo').value  # passer | shooter | solo

        self.declare_parameter('cmd_vel_topic', f'/{ns}/cmd_vel')
        self.declare_parameter('robot_pose_topic', f'/{mocap}/{ns}/pose')
        self.declare_parameter('stick_pose_topic', f'/{mocap}/stick1/pose')
        self.declare_parameter('puck_pose_topic',  f'/{mocap}/puck1/pose')
        self.declare_parameter('goal_pose_topic',  f'/{mocap}/goal1/pose')
        # teammate receive point (for 'passer'): a mocap marker or a fixed param
        self.declare_parameter('receive_pose_topic', f'/{mocap}/receive1/pose')
        # obstacles to avoid: list of mocap pose topics (other robots, static obstacles)
        self.obstacle_topics = self.declare_parameter(
            'obstacle_topics', [f'/{mocap}/robot_other/pose']).value

        # geometry / gains
        self.l = self.declare_parameter('l', 0.20).value
        self.l_stick = self.declare_parameter('l_stick', 0.45).value
        self.k = self.declare_parameter('k', 1.0).value
        self.u_max = self.declare_parameter('u_max', 0.5).value      # point speed cap (m/s)
        self.w_max = self.declare_parameter('w_max', 2.0).value

        # CBF / strategy
        self.R_safe = self.declare_parameter('R_safe', 0.45).value   # keep-out radius (m)
        self.alpha = self.declare_parameter('alpha', 2.0).value      # CBF class-K gain
        self.d_behind = self.declare_parameter('d_behind', 0.20).value
        self.d_through = self.declare_parameter('d_through', 0.25).value
        self.reach_tol = self.declare_parameter('reach_tol', 0.05).value
        self.goal_tol = self.declare_parameter('goal_tol', 0.15).value
        self.near_puck = self.declare_parameter('near_puck', 0.4).value  # 'shooter' wait gate

        self.robot = self.stick = self.puck = self.goal = self.receive = None
        self.obstacles: dict = {}
        self._busy = False
        self.phase = 'WAIT_PASS' if self.role == 'shooter' else 'GO_TO_STICK'

        self._sub('robot_pose_topic', self._on_robot)
        self._sub('stick_pose_topic', lambda m: setattr(self, 'stick', self._xy(m)))
        self._sub('puck_pose_topic',  lambda m: setattr(self, 'puck',  self._xy(m)))
        self._sub('goal_pose_topic',  lambda m: setattr(self, 'goal',  self._xy(m)))
        self._sub('receive_pose_topic', lambda m: setattr(self, 'receive', self._xy(m)))
        for t in self.obstacle_topics:
            self.create_subscription(PoseStamped, t,
                                     lambda m, key=t: self.obstacles.__setitem__(key, self._xy(m)), 10)

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.gripper = ActionClient(self, robomaster_msgs.action.GripperControl, f'/{ns}/gripper')
        self.timer = self.create_timer(0.05, self.step)
        if not _HAVE_QP:
            self.get_logger().warn('qpsolvers not importable; running WITHOUT the CBF safety filter!')
        self.get_logger().info(f'ECE687 hockey controller started (role={self.role}).')

    # ---- helpers ---------------------------------------------------------
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

    def unit(self, a, b):
        d = b - a
        n = np.linalg.norm(d)
        return d / n if n > 1e-6 else np.array([1.0, 0.0])

    # ---- CLF-CBF-QP safe navigation -------------------------------------
    def safe_u(self, p: np.ndarray, p_des: np.ndarray) -> np.ndarray:
        """Return a safe single-integrator input u for the control point p."""
        u_nom = -self.k * (p - p_des)
        # cap nominal speed
        n = np.linalg.norm(u_nom)
        if n > self.u_max:
            u_nom = u_nom * (self.u_max / n)

        obs = list(self.obstacles.values())
        if not _HAVE_QP or not obs:
            return u_nom

        # QP:  min 0.5 u^T P u + q^T u   s.t.  G u <= h
        P = 2.0 * np.eye(2)
        q = -2.0 * u_nom
        G_rows, h_rows = [], []
        for o in obs:
            grad_h = 2.0 * (p - o)                  # d/dp ||p-o||^2
            h_val = float(np.dot(p - o, p - o) - self.R_safe ** 2)
            # grad_h . u >= -alpha h   ->   -grad_h . u <= alpha h
            G_rows.append(-grad_h)
            h_rows.append(self.alpha * h_val)
        # input box |u| <= u_max
        G_rows += [[1, 0], [-1, 0], [0, 1], [0, -1]]
        h_rows += [self.u_max] * 4
        G = np.array(G_rows, dtype=float)
        h = np.array(h_rows, dtype=float)
        try:
            u = solve_qp(P, q, G, h, solver='quadprog')
            if u is not None:
                return u
        except Exception as e:
            self.get_logger().warn(f'QP failed ({e}); falling back to nominal.',
                                   throttle_duration_sec=2.0)
        return u_nom

    def drive_point_to(self, p_des: np.ndarray) -> float:
        x, y, th = self.robot
        p = self.control_point()
        u = self.safe_u(p, p_des)
        c, s = math.cos(th), math.sin(th)
        v = c * u[0] + s * u[1]
        w = (-s * u[0] + c * u[1]) / self.l           # approximate linearization
        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(max(-self.w_max, min(self.w_max, w)))
        self.cmd_pub.publish(tw)
        return float(np.linalg.norm(p_des - p))

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---- state machine ---------------------------------------------------
    def step(self) -> None:
        need = [self.robot, self.stick, self.puck, self.goal]
        if self.role == 'passer':
            need.append(self.receive)
        if any(o is None for o in need) or self._busy:
            return

        # target of the "shot": teammate receive point if passer, else the goal
        shot_target = self.receive if self.role == 'passer' else self.goal

        if self.phase == 'WAIT_PASS':                                  # shooter idles until puck arrives
            self.stop()
            if np.linalg.norm(self.puck - self.control_point()) < self.near_puck:
                self.get_logger().info('Puck arrived -> GO_TO_STICK')
                self.phase = 'GO_TO_STICK'

        elif self.phase == 'GO_TO_STICK':                             # T1
            if self.drive_point_to(self.stick) < self.reach_tol:
                self.stop(); self.phase = 'PICK_STICK'

        elif self.phase == 'PICK_STICK':                              # T2
            self.close_gripper()

        elif self.phase == 'GO_TO_PUCK':                              # T3
            u = self.unit(shot_target, self.puck)   # points from target back to puck side
            lineup = self.puck + self.d_behind * u  # stay on the far side of the puck from target
            if self.drive_point_to(lineup) < self.reach_tol:
                self.stop(); self.phase = 'SHOOT'

        elif self.phase == 'SHOOT':                                   # T4
            u = self.unit(self.puck, shot_target)
            self.drive_point_to(self.puck + self.d_through * u)
            if np.linalg.norm(self.puck - shot_target) < self.goal_tol:
                self.stop()
                self.get_logger().info('Puck delivered -> DONE'); self.phase = 'DONE'

        elif self.phase == 'DONE':
            self.stop()

    # ---- T2 gripper ------------------------------------------------------
    def close_gripper(self):
        self._busy = True
        self.gripper.wait_for_server()
        g = robomaster_msgs.action.GripperControl.Goal()
        g.target_state, g.power = 2, 0.7
        self.get_logger().info('Closing gripper...')
        self.gripper.send_goal_async(g).add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(self._grip_done))

    def _grip_done(self, _):
        self.l = self.l_stick
        self.get_logger().info('Stick grabbed -> GO_TO_PUCK')
        self.phase, self._busy = 'GO_TO_PUCK', False


def main() -> None:
    rclpy.init()
    node = ECE687Hockey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
