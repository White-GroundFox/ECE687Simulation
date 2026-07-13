import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math

# --- Mission Configuration ---
MY_ID = 3                    # robot being controlled
TARGET_ID = 6                # robot to approach
STANDOFF_DISTANCE = 2.0      # [m] distance to keep between the two robots
APPROACH_BEARING_DEG = 0.0   # approach direction in the TARGET's frame:
                             #   0 = stand in front of it (face-to-face)
                             #  90 = stand on its left side, 180 = behind it, etc.


class ApproachRobotNode(Node):
    """
    Drives robot MY_ID to a standoff pose relative to robot TARGET_ID using the
    approximate linearization method: control the point p located L_OFFSET ahead
    of the robot, then map p-dot to (v, w) through L^-1(l) R^T(theta).
    Phases: NAVIGATE -> ALIGN -> HOLD (re-navigates if the target moves away).
    """

    def __init__(self):
        super().__init__('approach_robot_node')

        # --- Controller Settings ---
        self._l = 0.15            # [m] offset of the controlled point p (approx. linearization)
        self._kp_pos = 0.8        # proportional gain on p error
        self._kp_ang = 1.5        # proportional gain for final in-place alignment
        self._v_max = 0.5         # [m/s] safety clamp
        self._w_max = 1.5         # [rad/s] safety clamp
        self._pos_tol = 0.05      # [m] position tolerance
        self._ang_tol = 0.05      # [rad] heading tolerance (~3 deg)

        # --- State ---
        self.my_pose = None       # np.array([x, y, theta])
        self.target_pose = None
        self.phase = 'NAVIGATE'

        mocap_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(
            PoseStamped, f'/vrpn_mocap/dji_robot_{MY_ID}/pose',
            lambda msg: self._pose_callback(msg, 'my_pose'), mocap_qos)
        self.create_subscription(
            PoseStamped, f'/vrpn_mocap/dji_robot_{TARGET_ID}/pose',
            lambda msg: self._pose_callback(msg, 'target_pose'), mocap_qos)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/robot{MY_ID}/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f'Approach node started: robot {MY_ID} -> robot {TARGET_ID}, '
            f'standoff {STANDOFF_DISTANCE} m, bearing {APPROACH_BEARING_DEG} deg')

    def _pose_callback(self, msg, attr):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        setattr(self, attr, np.array([msg.pose.position.x, msg.pose.position.y, yaw]))

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _compute_goal_pose(self):
        """Standoff pose relative to the target's CURRENT pose, so the goal
        follows the target if it moves. The goal heading points back at the
        target: face-to-face when APPROACH_BEARING_DEG = 0."""
        tx, ty, tth = self.target_pose
        bearing = tth + math.radians(APPROACH_BEARING_DEG)
        gx = tx + STANDOFF_DISTANCE * math.cos(bearing)
        gy = ty + STANDOFF_DISTANCE * math.sin(bearing)
        gth = self._wrap_angle(bearing + math.pi)
        return np.array([gx, gy, gth])

    def control_loop(self):
        if self.my_pose is None or self.target_pose is None:
            self.get_logger().info('Waiting for mocap poses...', throttle_duration_sec=2.0)
            return

        goal = self._compute_goal_pose()
        x, y, th = self.my_pose
        pos_err = np.linalg.norm(goal[:2] - self.my_pose[:2])
        ang_err = self._wrap_angle(goal[2] - th)

        cmd = Twist()

        if self.phase == 'NAVIGATE':
            # p = point l ahead of the robot; drive it to the same point of the goal pose
            c, s = math.cos(th), math.sin(th)
            p = np.array([x + self._l * c, y + self._l * s])
            p_goal = goal[:2] + self._l * np.array([math.cos(goal[2]), math.sin(goal[2])])

            p_dot = self._kp_pos * (p_goal - p)
            speed = np.linalg.norm(p_dot)
            if speed > self._v_max:
                p_dot *= self._v_max / speed

            # [v, w]^T = L^-1(l) R^T(theta) p_dot
            v = c * p_dot[0] + s * p_dot[1]
            w = (-s * p_dot[0] + c * p_dot[1]) / self._l

            cmd.linear.x = float(np.clip(v, -self._v_max, self._v_max))
            cmd.angular.z = float(np.clip(w, -self._w_max, self._w_max))

            if pos_err < self._pos_tol:
                self.get_logger().info(f'Position reached (err {pos_err:.3f} m). Aligning heading...')
                self.phase = 'ALIGN'

        elif self.phase == 'ALIGN':
            cmd.angular.z = float(np.clip(self._kp_ang * ang_err, -self._w_max, self._w_max))
            if abs(ang_err) < self._ang_tol:
                self.get_logger().info(f'Aligned (err {math.degrees(ang_err):.1f} deg). Holding pose.')
                self.phase = 'HOLD'

        elif self.phase == 'HOLD':
            # zero command; re-engage if the target robot moved away
            if pos_err > 3 * self._pos_tol or abs(ang_err) > 3 * self._ang_tol:
                self.get_logger().info('Target moved. Re-navigating...')
                self.phase = 'NAVIGATE'

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    # keep the ROS context alive on Ctrl+C so the safety stop in `finally`
    # can still publish; rclpy's own SIGINT handler would shut it down first
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ApproachRobotNode()

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
