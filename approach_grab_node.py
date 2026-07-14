#!/usr/bin/env python3
"""
approach_grab_node.py -- MERGE of the teammate's gripper controller
(move_robot_node with GripperControl) and the phased approach controller
(approach_robot_node.py sim-tested, approach_stick_node.py lab port).

What each parent contributed:
  * teammate's node : --robot_id CLI, /robot{N} topic layout, GripperControl
                      action flow (open on the way in, close on arrival).
  * approach nodes  : unicycle + approximate linearization (Sec 1.1 eqs 2-3),
                      NAVIGATE -> REFINE -> ALIGN -> ADVANCE phase machine,
                      speed clamps, Ctrl+C safety stop, line-of-sight standoff
                      that works with arbitrary/unknown stick yaw.
  * stick_grabber   : arm grasp posture (x = reach, z = height) held via
                      /robotN/target_arm_position, gripper busy/retry pattern.

Mission: drive robot N to a standoff pose `standoff` metres from the stick,
end FACE TO FACE with it, advance straight until `approach` metres away,
then close the gripper on it.

  NAVIGATE -> REFINE -> ALIGN -> ADVANCE -> GRASP -> DONE
  (gripper opens once at the start; arm goes to grasp posture from REFINE on)

Unlike the robot-vs-robot sim node there is NO re-navigate after DONE: once
the stick is in the gripper it moves WITH the robot, so chasing it would
chase ourselves.

Run (inside the container, robot + vrpn_mocap up):
  python3 /hockey/approach_grab_node.py --robot_id 3 --ros-args \
    -p robot_pose_topic:=/vrpn_mocap/dji_robot_3/pose \
    -p standoff:=0.50 -p approach:=0.30 -p v_max:=0.10

Lab notes (from TEST_RUNBOOK.md):
  * robot namespace number and mocap body number do NOT necessarily match --
    the STEP 1 nudge test tells you the dji_robot_N number.
  * if the robot circles instead of converging, the Vicon body x-axis is not
    the robot's forward direction: set yaw_offset_deg (find_my_body.py).
  * calibrate arm_z first: ros2 topic echo /robot3/arm_position (STEP 4).
  * tune `approach` in ~5 cm steps until the open gripper straddles the
    stick; keep it < standoff. Oscillates: lower v_max to 0.07 / kp_pos 0.5.
"""

import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped, Point
from robomaster_msgs.action import GripperControl

# gripper target_state values (robomaster_ros): 0=pause, 1=open, 2=close
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ApproachGrabNode(Node):
    def __init__(self, robot_id: int):
        super().__init__('approach_grab_node')
        self.robot_name = f'/robot{robot_id}'

        # ---- topics ----------------------------------------------------------
        self.declare_parameter('robot_pose_topic',
                               f'/vrpn_mocap/dji_robot_{robot_id}/pose')
        self.declare_parameter('target_topic', '/vrpn_mocap/hockey_sticks_1/pose')

        # ---- mission ---------------------------------------------------------
        self.standoff = self.declare_parameter('standoff', 0.50).value  # m, body center to stick
        self.approach = self.declare_parameter('approach', 0.30).value  # m, final grasp spacing
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

        # ---- arm / gripper ----------------------------------------------------
        # grasp posture in the arm's forward-vertical plane; CALIBRATE arm_z to
        # the stick height (see stick_grabber_node.py / TEST_RUNBOOK STEP 4)
        self.set_arm = self.declare_parameter('set_arm', True).value
        self.arm_x = self.declare_parameter('arm_x', 0.18).value
        self.arm_z = self.declare_parameter('arm_z', 0.02).value
        self.grip_power = self.declare_parameter('grip_power', 0.5).value

        # ---- state -----------------------------------------------------------
        self.my_pose = None      # np.array([x, y, theta])
        self.target_pose = None  # np.array([x, y, theta])
        self.phase = 'NAVIGATE'
        self._gripper_busy = False    # a gripper action is in flight
        self._gripper_opened = False

        mocap_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(
            PoseStamped, self.get_parameter('robot_pose_topic').value,
            lambda msg: self._pose_callback(msg, 'my_pose'), mocap_qos)
        self.create_subscription(
            PoseStamped, self.get_parameter('target_topic').value,
            lambda msg: self._pose_callback(msg, 'target_pose'), mocap_qos)

        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.robot_name}/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Point, f'{self.robot_name}/target_arm_position', 10)

        self.action_group = ReentrantCallbackGroup()
        self.gripper = ActionClient(self, GripperControl, f'{self.robot_name}/gripper',
                                    callback_group=self.action_group)
        self.get_logger().info('Connecting to gripper action server...')
        if not self.gripper.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper server not found; will retry in loop.')

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info(
            f"approach_grab_node: {self.get_parameter('robot_pose_topic').value} -> "
            f"{self.get_parameter('target_topic').value}, standoff {self.standoff} m, "
            f"approach {self.approach} m, "
            f"mode={'target-yaw' if self.use_target_yaw else 'line-of-sight'}")

    # ---------------------------------------------------------------- mocap I/O
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
            # line-of-sight: park on the segment target -> current robot pos;
            # robust when the stick's Vicon yaw is arbitrary/unknown
            dx, dy = self.my_pose[0] - tx, self.my_pose[1] - ty
            if math.hypot(dx, dy) < 1e-6:
                return None
            bearing = math.atan2(dy, dx)
        gx = tx + self.standoff * math.cos(bearing)
        gy = ty + self.standoff * math.sin(bearing)
        gth = wrap(bearing + math.pi)   # look back at the target
        return np.array([gx, gy, gth])

    # ------------------------------------------------------------- control loop
    def control_loop(self):
        if self.my_pose is None or self.target_pose is None:
            self.get_logger().info('Waiting for mocap poses...', throttle_duration_sec=2.0)
            return

        # open the gripper once, early, so it is already open around the stick
        # on arrival (teammate's flow); driving does not wait on it
        if not self._gripper_opened and not self._gripper_busy:
            self._send_gripper(GRIPPER_OPEN)

        goal = self._compute_goal_pose()
        if goal is None:
            return
        x, y, th = self.my_pose
        dist_to_target = float(np.linalg.norm(self.target_pose[:2] - self.my_pose[:2]))
        # heading error to face the TARGET from the current position; using the
        # goal pose's heading instead would point at the target only if the
        # position error were exactly zero
        ang_err = wrap(math.atan2(self.target_pose[1] - y,
                                  self.target_pose[0] - x) - th)

        # hold the arm in the grasp posture from REFINE onward so the open
        # gripper is at stick height before the final advance (re-publishing
        # every tick makes the driver hold it, as in stick_grabber_node)
        if self.set_arm and self.phase in ('REFINE', 'ALIGN', 'ADVANCE', 'GRASP'):
            self.arm_pub.publish(Point(x=float(self.arm_x), y=0.0, z=float(self.arm_z)))

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

            # converge on the CONTROLLED POINT: p can land on p_goal with the body
            # at any heading, in which case v,w -> 0 and a body-center check
            # can deadlock here
            p_err = float(np.linalg.norm(p_goal - p))
            if p_err < self.pos_tol:
                self.get_logger().info(
                    f'Point reached (err {p_err:.3f} m). Refining standoff position...')
                self.phase = 'REFINE'

        elif self.phase == 'REFINE':
            # trim the body-center residual the p-controller leaves behind (up to
            # ~l sideways); without this, ALIGN freezes that offset into a 5-15 deg
            # skew off the approach axis. Aim at the STANDOFF POINT, not the
            # target: v is gated by cos(aim error), so a misaimed robot rotates in
            # place and an aimed one drives straight -- pure unicycle moves
            vec = goal[:2] - self.my_pose[:2]
            center_err = float(np.linalg.norm(vec))
            if center_err < self.pos_tol:
                self.get_logger().info(
                    f'Standoff point reached (err {center_err:.3f} m). Aligning heading...')
                self.phase = 'ALIGN'
            else:
                aim_err = wrap(math.atan2(vec[1], vec[0]) - th)
                cmd.linear.x = float(np.clip(
                    self.kp_pos * center_err * max(0.0, math.cos(aim_err)),
                    0.0, self.v_max))
                cmd.angular.z = float(np.clip(
                    self.kp_ang * aim_err, -self.w_max, self.w_max))

        elif self.phase == 'ALIGN':
            cmd.angular.z = float(np.clip(self.kp_ang * ang_err, -self.w_max, self.w_max))
            if abs(ang_err) < self.ang_tol:
                self.get_logger().info(
                    f'Face-to-face (err {math.degrees(ang_err):.1f} deg). '
                    f'Advancing to {self.approach} m...')
                self.phase = 'ADVANCE'

        elif self.phase == 'ADVANCE':
            # straight final approach: forward speed on the remaining spacing,
            # yaw only correcting drift so the nose stays on the stick
            cmd.linear.x = float(np.clip(
                self.kp_pos * (dist_to_target - self.approach),
                -self.v_max, self.v_max))
            cmd.angular.z = float(np.clip(self.kp_ang * ang_err, -self.w_max, self.w_max))
            if abs(dist_to_target - self.approach) < self.pos_tol:
                self.get_logger().info(
                    f'Grasp distance reached ({dist_to_target:.2f} m). Closing gripper...')
                self.phase = 'GRASP'

        elif self.phase == 'GRASP':
            # robot stopped (cmd stays zero); close on the stick once
            if self._gripper_opened and not self._gripper_busy:
                self._send_gripper(GRIPPER_CLOSE)

        elif self.phase == 'DONE':
            pass   # stick in hand; hold zero command

        self.get_logger().info(
            f'{self.phase}: dist {dist_to_target:.2f} m, '
            f'ang_err {math.degrees(ang_err):.0f} deg',
            throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(cmd)

    # ----------------------------------------------------------- gripper action
    def _send_gripper(self, state_value: int):
        if not self.gripper.server_is_ready():
            self.get_logger().warn('Gripper server not ready; retrying...',
                                   throttle_duration_sec=2.0)
            return  # try again next tick
        self._gripper_busy = True
        goal = GripperControl.Goal()
        goal.target_state = state_value
        goal.power = float(self.grip_power)
        self.get_logger().info(
            f'Gripper -> {"OPEN" if state_value == GRIPPER_OPEN else "CLOSE"}')
        self.gripper.send_goal_async(goal).add_done_callback(self._gripper_response)

    def _gripper_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gripper goal rejected; retrying.')
            self._gripper_busy = False
            return
        handle.get_result_async().add_done_callback(self._gripper_done)

    def _gripper_done(self, _):
        self._gripper_busy = False
        if not self._gripper_opened:
            self._gripper_opened = True
            self.get_logger().info('Gripper open. Approaching...')
        elif self.phase == 'GRASP':
            self.get_logger().info('Stick grasped. DONE.')
            self.phase = 'DONE'

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_id', type=int, required=True,
                        help='ID of the robot to control (namespace /robotN)')
    cli, remaining = parser.parse_known_args(args)

    # keep the ROS context alive on Ctrl+C so the safety stop in `finally`
    # can still publish; rclpy's own SIGINT handler would shut it down first
    rclpy.init(args=remaining, signal_handler_options=SignalHandlerOptions.NO)
    node = ApproachGrabNode(robot_id=cli.robot_id)

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
