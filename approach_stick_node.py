#!/usr/bin/env python3
"""
approach_stick_node.py -- LAB PORT of the sim-tested ApproachRobotNode
(White-GroundFox/ECE687Simulation, approach_robot_node.py).

Mission: drive the robot to a standoff pose near a mocap target (default:
hockey_sticks_1) and end FACE TO FACE with it -- chassis heading pointing at
the target. Unicycle + approximate linearization (Sec. 1.1, eqs. 2-3), the
same math as move_robot_node.py; the NAVIGATE -> ALIGN -> HOLD phase machine
is carried over from the sim node.

Changes vs the sim version (integration to hardware):
  * every hardcoded constant (MY_ID, TARGET_ID, standoff, gains) is now a ROS
    parameter. In the lab the robot namespace (robot1) and its mocap body
    (dji_robot_N) do NOT necessarily match -- both topics are parameters
    (STEP 1 nudge test in TEST_RUNBOOK.md tells you N).
  * the target is any PoseStamped mocap body (a stick), not only a robot.
  * NEW `use_target_yaw` parameter:
      false = DEFAULT, robust for sticks: goal point lies `standoff` metres
              short of the target along the current robot->target line, and
              the goal heading faces the target. Works with arbitrary /
              unknown yaw on the stick's Vicon body.
      true  = sim behavior: standoff point in the TARGET's frame at
              bearing_deg (0 = in front of its +x face, 90 = its left, ...),
              heading pointing back at it. Use once you trust the stick yaw.
  * hardware-safe default speeds (v_max 0.10, like TEST_RUNBOOK STEP 3).

Run (inside the container, robot + vrpn_mocap up):
  python3 /hockey/approach_stick_node.py --ros-args \
    -p robot:=robot1 \
    -p robot_pose_topic:=/vrpn_mocap/dji_robot_1/pose \
    -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
    -p standoff:=0.50 -p v_max:=0.10
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ApproachStickNode(Node):
    def __init__(self):
        super().__init__('approach_stick_node')

        # ---- topics ----------------------------------------------------------
        robot = self.declare_parameter('robot', 'robot1').value
        self.declare_parameter('cmd_vel_topic', f'/{robot}/cmd_vel')
        self.declare_parameter('robot_pose_topic', '/vrpn_mocap/dji_robot_1/pose')
        self.declare_parameter('target_topic', '/vrpn_mocap/hockey_sticks_1/pose')

        # ---- mission ---------------------------------------------------------
        self.standoff = self.declare_parameter('standoff', 0.50).value  # m, body center to target
        self.use_target_yaw = self.declare_parameter('use_target_yaw', False).value
        self.bearing_deg = self.declare_parameter('bearing_deg', 0.0).value  # only if use_target_yaw

        # ---- gains / limits (sim values softened for hardware) ---------------
        self.l = self.declare_parameter('l', 0.15).value           # offset of point p (m)
        self.kp_pos = self.declare_parameter('kp_pos', 0.7).value  # P-gain on p
        self.kp_ang = self.declare_parameter('kp_ang', 1.2).value  # P-gain, in-place align
        self.v_max = self.declare_parameter('v_max', 0.10).value   # m/s
        self.w_max = self.declare_parameter('w_max', 1.0).value    # rad/s
        self.pos_tol = self.declare_parameter('pos_tol', 0.05).value  # m
        self.ang_tol = self.declare_parameter('ang_tol', 0.05).value  # rad (~3 deg)
        # correction if the robot's Vicon body x-axis is not the robot's forward
        # direction (measure it with find_my_body.py; wrong heading = circling)
        self.yaw_offset = math.radians(
            self.declare_parameter('yaw_offset_deg', 0.0).value)

        # ---- state -----------------------------------------------------------
        self.my_pose = None      # np.array([x, y, theta])
        self.target_pose = None  # np.array([x, y, theta])
        self.phase = 'NAVIGATE'

        mocap_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(
            PoseStamped, self.get_parameter('robot_pose_topic').value,
            lambda msg: self._pose_callback(msg, 'my_pose'), mocap_qos)
        self.create_subscription(
            PoseStamped, self.get_parameter('target_topic').value,
            lambda msg: self._pose_callback(msg, 'target_pose'), mocap_qos)

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"approach_stick_node: {self.get_parameter('robot_pose_topic').value} -> "
            f"{self.get_parameter('target_topic').value}, standoff {self.standoff} m, "
            f"mode={'target-yaw' if self.use_target_yaw else 'line-of-sight'}")

    def _pose_callback(self, msg: PoseStamped, attr: str):
        yaw = yaw_from_quat(msg.pose.orientation)
        if attr == 'my_pose':
            yaw = wrap(yaw + self.yaw_offset)
        setattr(self, attr,
                np.array([msg.pose.position.x, msg.pose.position.y, yaw]))

    def _compute_goal_pose(self):
        """Standoff pose relative to the target's CURRENT position, recomputed
        every tick so the goal follows a moving target. Goal heading always
        points at the target: face-to-face on arrival."""
        tx, ty, tth = self.target_pose
        if self.use_target_yaw:
            # sim behavior: park at bearing_deg in the target's own frame
            bearing = tth + math.radians(self.bearing_deg)
        else:
            # line-of-sight: park on the segment target -> current robot pos
            dx, dy = self.my_pose[0] - tx, self.my_pose[1] - ty
            if math.hypot(dx, dy) < 1e-6:
                return None
            bearing = math.atan2(dy, dx)
        gx = tx + self.standoff * math.cos(bearing)
        gy = ty + self.standoff * math.sin(bearing)
        gth = wrap(bearing + math.pi)   # look back at the target
        return np.array([gx, gy, gth])

    def control_loop(self):
        if self.my_pose is None or self.target_pose is None:
            self.get_logger().info('Waiting for mocap poses...', throttle_duration_sec=2.0)
            return

        goal = self._compute_goal_pose()
        if goal is None:
            return
        x, y, th = self.my_pose
        pos_err = float(np.linalg.norm(goal[:2] - self.my_pose[:2]))
        ang_err = wrap(goal[2] - th)

        cmd = Twist()

        if self.phase == 'NAVIGATE':
            # p = point l ahead of the robot; drive it to the same point of the goal pose
            c, s = math.cos(th), math.sin(th)
            p = np.array([x + self.l * c, y + self.l * s])
            p_goal = goal[:2] + self.l * np.array([math.cos(goal[2]), math.sin(goal[2])])

            p_dot = self.kp_pos * (p_goal - p)
            speed = np.linalg.norm(p_dot)
            if speed > self.v_max:
                p_dot *= self.v_max / speed

            # [v, w]^T = L^-1(l) R^T(theta) p_dot   (eq. 3)
            v = c * p_dot[0] + s * p_dot[1]
            w = (-s * p_dot[0] + c * p_dot[1]) / self.l

            cmd.linear.x = float(np.clip(v, -self.v_max, self.v_max))
            cmd.linear.y = 0.0                      # unicycle: never strafe
            cmd.angular.z = float(np.clip(w, -self.w_max, self.w_max))

            if pos_err < self.pos_tol:
                self.get_logger().info(f'Position reached (err {pos_err:.3f} m). Aligning...')
                self.phase = 'ALIGN'

        elif self.phase == 'ALIGN':
            cmd.angular.z = float(np.clip(self.kp_ang * ang_err, -self.w_max, self.w_max))
            if abs(ang_err) < self.ang_tol:
                self.get_logger().info(
                    f'Face-to-face (err {math.degrees(ang_err):.1f} deg). Holding.')
                self.phase = 'HOLD'

        elif self.phase == 'HOLD':
            # zero command; re-engage if the target (or we) drifted
            if pos_err > 3 * self.pos_tol or abs(ang_err) > 3 * self.ang_tol:
                self.get_logger().info('Target moved. Re-navigating...')
                self.phase = 'NAVIGATE'

        self.get_logger().info(
            f'{self.phase}: pos_err {pos_err:.2f} m, ang_err {math.degrees(ang_err):.0f} deg',
            throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    # keep the ROS context alive on Ctrl+C so the safety stop in `finally`
    # can still publish; rclpy's own SIGINT handler would shut it down first
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ApproachStickNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt: stopping robot...')
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
