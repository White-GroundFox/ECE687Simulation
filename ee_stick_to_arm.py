#!/usr/bin/env python3
"""
ee_stick_to_arm.py  -- teammate's EEControllerNode + the world->arm transform.

FUNCTION: express the hockey stick (published in the 'world' frame by mocap) in
the robot's 'arm_base_link' frame, so it can be compared to arm_position / used
as a move_arm target.

FRAME CHAIN (why we need the robot's own pose):
    world --(robot mocap pose)--> base_link --(fixed offset)--> arm_base_link
  * stick in world:      /vrpn_mocap/hockey_sticks_1/pose
  * robot in world:      /vrpn_mocap/dji_robot_N/pose   <-- MUST set N (nudge test)
  * base_link->arm_base_link offset (from robomaster URDF arm_base_joint):
        xyz = (0, 0.0010384, 0.0906477)  ->  off_x~0, off_z~0.0906 m

REACHABILITY REALITY CHECK:
  The arm moves the gripper only in its x(reach)-z(height) plane, |reach| ~0.2 m,
  and cannot move sideways. So the transform only yields a *usable* arm target
  when the CHASSIS has already driven the robot up to the stick and centred it
  (y_arm ~ 0). Until then this node just reports the stick's arm-frame position
  and tells you what the chassis still has to do. When the target is reachable
  AND laterally aligned, it commands move_arm.

Run (after identifying N and, ideally, calibrating off_z):
  python3 /hockey/ee_stick_to_arm.py --robot_id 3 --ros-args \
    -p robot_pose_topic:=/vrpn_mocap/dji_robot_2/pose \
    -p off_z:=0.0906 -p send_arm:=false     # keep false until you trust the numbers
"""

import math
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, PoseStamped
from robomaster_msgs.action import MoveArm


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EEStickToArm(Node):
    def __init__(self, robot_id):
        super().__init__('ee_stick_to_arm')
        self.robot_name = f"/robot{robot_id}"

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('control_frequency', 5.0)
        # robot's mocap rigid body -- SET THIS to the dji_robot_N that is your robot
        self.declare_parameter('robot_pose_topic', '/vrpn_mocap/dji_robot_2/pose')
        self.declare_parameter('stick_pose_topic', '/vrpn_mocap/hockey_sticks_1/pose')
        # static base_link -> arm_base_link offset (from URDF arm_base_joint)
        self.declare_parameter('off_x', 0.0)
        self.declare_parameter('off_z', 0.0906)
        # arm workspace / alignment gates
        self.declare_parameter('reach_max', 0.20)   # max forward reach (m)
        self.declare_parameter('y_tol', 0.05)       # lateral alignment tol (m)
        self.declare_parameter('send_arm', False)   # actually command move_arm?

        self.control_frequency = self.get_parameter('control_frequency').value
        self.off_x = self.get_parameter('off_x').value
        self.off_z = self.get_parameter('off_z').value
        self.reach_max = self.get_parameter('reach_max').value
        self.y_tol = self.get_parameter('y_tol').value
        self.send_arm = self.get_parameter('send_arm').value

        # ---- state ----------------------------------------------------------
        self.robot_pose = None   # (x, y, z, yaw) in world
        self.stick_pose = None   # (x, y, z) in world
        self.ee_position = None
        self._goal_in_flight = False
        self._done = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, self.get_parameter('robot_pose_topic').value,
                                 self.robot_pose_callback, qos)
        self.create_subscription(PoseStamped, self.get_parameter('stick_pose_topic').value,
                                 self.hockey_stick_pos_callback, qos)
        self.create_subscription(PointStamped, f'{self.robot_name}/arm_position',
                                 self.arm_position_callback, qos)

        self.action_group = ReentrantCallbackGroup()
        self.move_arm_client = ActionClient(self, MoveArm, f'{self.robot_name}/move_arm',
                                            callback_group=self.action_group)

        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)
        self.get_logger().info(f'ee_stick_to_arm up for {self.robot_name}. send_arm={self.send_arm}')

    # ------------------------------------------------------------------ I/O
    def robot_pose_callback(self, msg):
        p = msg.pose.position
        self.robot_pose = (p.x, p.y, p.z, yaw_from_quat(msg.pose.orientation))

    def hockey_stick_pos_callback(self, msg):
        p = msg.pose.position
        self.stick_pose = (p.x, p.y, p.z)

    def arm_position_callback(self, msg):
        self.ee_position = msg

    # --------------------------------------------- THE TRANSFORM (the function)
    def transform_to_arm_frame(self):
        """Return the stick position expressed in arm_base_link: (x, y, z) metres.
        x = forward reach, y = left (arm can't use it), z = height. None if data missing."""
        if self.robot_pose is None or self.stick_pose is None:
            return None
        xr, yr, zr, yaw = self.robot_pose
        xs, ys, zs = self.stick_pose

        # 1) world -> base_link : translate to robot, rotate by -yaw
        dx, dy = xs - xr, ys - yr
        c, s = math.cos(yaw), math.sin(yaw)
        x_base = c * dx + s * dy
        y_base = -s * dx + c * dy
        z_base = zs - zr

        # 2) base_link -> arm_base_link : subtract the fixed offset (no rotation)
        x_arm = x_base - self.off_x
        y_arm = y_base
        z_arm = z_base - self.off_z
        return (x_arm, y_arm, z_arm)

    # ------------------------------------------------------------------ loop
    def control_loop(self):
        arm = self.transform_to_arm_frame()
        if arm is None:
            self.get_logger().warn('waiting for robot pose + stick pose...',
                                   throttle_duration_sec=2.0)
            return
        x_arm, y_arm, z_arm = arm
        reach = math.hypot(x_arm, z_arm)
        self.get_logger().info(
            f'stick in arm frame: x={x_arm:.3f} y={y_arm:.3f} z={z_arm:.3f} '
            f'(reach={reach:.3f} m)')

        # reachability + alignment gates -- this is what the chassis must fix
        if abs(y_arm) > self.y_tol:
            self.get_logger().info(
                f'  -> NOT centred: chassis must move sideways by {y_arm:+.2f} m', throttle_duration_sec=1.0)
            return
        if x_arm <= 0.0 or reach > self.reach_max:
            self.get_logger().info(
                '  -> OUT OF ARM REACH: chassis must drive closer', throttle_duration_sec=1.0)
            return

        # reachable & aligned
        self.get_logger().info('  -> reachable & aligned.')
        if self.send_arm and not self._goal_in_flight and not self._done:
            self._send_move_arm(x_arm, z_arm)

    # ------------------------------------------------------- move_arm action
    def _send_move_arm(self, x, z):
        if not self.move_arm_client.server_is_ready():
            self.move_arm_client.wait_for_server(timeout_sec=0.5)
            return
        goal = MoveArm.Goal()
        goal.x, goal.z, goal.relative = float(x), float(z), False
        self._goal_in_flight = True
        self.get_logger().info(f'move_arm -> x={goal.x:.3f}, z={goal.z:.3f}')
        self.move_arm_client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('move_arm rejected.')
            self._goal_in_flight = False
            return
        gh.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, _):
        self.get_logger().info('move_arm finished.')
        self._goal_in_flight = False
        self._done = True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_id', type=int, default=3)
    args, remaining = parser.parse_known_args()
    rclpy.init(args=remaining)
    node = EEStickToArm(robot_id=args.robot_id)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
