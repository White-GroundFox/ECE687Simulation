#!/usr/bin/env python3
"""
ee_controller_node.py  -- STEP 1: move the arm end-effector to a target point.

Based on the teammate's EEControllerNode skeleton, with the TODOs resolved
against the official arm module docs and the MoveArm action definition:

  robomaster_msgs/action/MoveArm.action
    float32 x         # target x in metres (x forward)   -- arm_base_link frame
    float32 z         # target z in metres (z up)        -- arm_base_link frame
    bool    relative  # False = absolute wrt arm_base_link
    ---
    float32 progress  # 0..1 feedback

KEY IDEA (why there is no world->arm conversion here):
  The arm moves the gripper only in the robot's forward-vertical (x,z) plane.
  With relative=False, (x,z) is already expressed in arm_base_link. Horizontal
  placement toward the stick is the CHASSIS's job (a separate node), so this
  node's target is just a calibrated grasp point (target_x, target_z).

STEP 1 goal: command the EE to (target_x, target_z) once, using the move_arm
action, and log the live arm_position feedback. Later steps will drive the
target from the task state machine.
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from robomaster_msgs.action import MoveArm


class EEControllerNode(Node):
    def __init__(self, robot_id):
        super().__init__('ee_controller_node')
        self.robot_name = f"/robot{robot_id}"

        # ---- parameters -----------------------------------------------------
        self.declare_parameter('control_frequency', 10.0)
        # Target EE point in arm_base_link (metres). CALIBRATE to grasp height:
        #   echo /robotX/arm_position while nudging the arm to grasp posture.
        self.declare_parameter('target_x', 0.18)   # forward reach
        self.declare_parameter('target_z', 0.02)   # height
        self.declare_parameter('relative', False)  # absolute wrt arm_base_link

        self.control_frequency = self.get_parameter('control_frequency').value
        self.target_x = self.get_parameter('target_x').value
        self.target_z = self.get_parameter('target_z').value
        self.relative = self.get_parameter('relative').value

        # ---- state ----------------------------------------------------------
        self.ee_position = None      # latest PointStamped from arm_position
        self._goal_in_flight = False # a move_arm goal is being executed
        self._done = False           # target reached (step 1 = single shot)

        # ---- QoS ------------------------------------------------------------
        # arm_position is published by the driver with default RELIABLE/VOLATILE,
        # so a RELIABLE subscriber matches. (Only mocap topics need BEST_EFFORT.)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- interfaces -----------------------------------------------------
        self.create_subscription(
            PointStamped, f'{self.robot_name}/arm_position',
            self.arm_position_callback, qos)

        self.action_group = ReentrantCallbackGroup()
        self.move_arm_client = ActionClient(
            self, MoveArm, f'{self.robot_name}/move_arm',
            callback_group=self.action_group)

        self.get_logger().info('Waiting for move_arm action server...')
        if not self.move_arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('move_arm server not available yet; will retry in loop.')

        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f'EEControllerNode up for robot{robot_id} @ {self.control_frequency} Hz; '
            f'target (x={self.target_x}, z={self.target_z}), relative={self.relative}')

    # ------------------------------------------------------------------ loop
    def control_loop(self):
        # STEP 1: send the target exactly once, then wait for the result.
        if self._done or self._goal_in_flight:
            return
        if not self.move_arm_client.server_is_ready():
            self.move_arm_client.wait_for_server(timeout_sec=0.5)
            return

        goal = MoveArm.Goal()
        goal.x = float(self.target_x)     # metres, forward (arm_base_link)
        goal.z = float(self.target_z)     # metres, up      (arm_base_link)
        goal.relative = bool(self.relative)

        self._goal_in_flight = True
        self.get_logger().info(
            f'Sending move_arm goal: x={goal.x:.3f}, z={goal.z:.3f}, relative={goal.relative}')
        self.move_arm_client.send_goal_async(
            goal, feedback_callback=self.move_arm_feedback
        ).add_done_callback(self.move_arm_goal_response)

    # ------------------------------------------------------- action callbacks
    def move_arm_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('move_arm goal rejected; will retry.')
            self._goal_in_flight = False
            return
        self.get_logger().info('move_arm goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.move_arm_result_callback)

    def move_arm_feedback(self, feedback_msg):
        self.get_logger().info(f'move_arm progress: {feedback_msg.feedback.progress:.2f}')

    def move_arm_result_callback(self, future):
        self.get_logger().info('move_arm finished.')
        if self.ee_position is not None:
            p = self.ee_position.point
            self.get_logger().info(f'EE now at x={p.x:.3f}, z={p.z:.3f}')
        self._goal_in_flight = False
        self._done = True     # STEP 1 complete (single shot)

    # ------------------------------------------------------- subscription
    def arm_position_callback(self, msg):
        self.ee_position = msg   # PointStamped in arm_base_link; .point.x/.z in m


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot_id', type=int, default=1, help='ID of the robot to control')
    args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = EEControllerNode(robot_id=args.robot_id)
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
