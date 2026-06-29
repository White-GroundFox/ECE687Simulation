import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import JointState
from robomaster_msgs.action import GripperControl
import numpy as np
import math

class StickGrabberNode(Node):
    def __init__(self):
        super().__init__('stick_grabber_node')
        self.get_logger().info("Stick Grabber Node initialized.")
        
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # --- Constants and Geometry for the Robot ---
        # using the parameter echo: ros2 param get /robot3/robot_state_publisher robot_description
        # TODO: Get physical dimensions
        self._a1 = 0.22
        self._a2 = 0.15
        
        # --- Controller Settings ---
        self._error_threshold = 0.04  # 4 cm threshold
        self._kp = 1.5                 # Proportional Control Gain
        self._max_velocity = 0.25      # Safety clamp for velocity vectors (m/s)
        
        # --- State Tracking Variables ---
        self.joint_positions = None
        self.joint_velocities = None
        self.desired_pose = None
        self.robot_pose = None
        self.robot_pose_orientation = None

        # State Machine Phases:
        # 0: INIT_OPEN_GRIPPER
        # 1: MOVE_TO_STICK
        # 2: CLOSE_GRIPPER
        # 3: DONE
        self._currentPhase = 0
        self._gripper_action_running = False

        # --- Subscriptions ---
        self.create_subscription(PoseStamped, '/vrpn_mocap/hockey_sticks_1/pose', self.stick_mocap_callback, mocap_qos)  
        self.create_subscription(JointState, '/robot3/joint_states', self.joint_states_callback, mocap_qos)
        self.create_subscription(PoseStamped, '/vrpn_mocap/dji_robot_3/pose', self.robot_pose_callback, mocap_qos)

        # --- Action Clients & Publishers ---
        self.cb_group = ReentrantCallbackGroup()
        self.gripper_client = ActionClient(self, GripperControl, '/robot3/gripper', callback_group=self.cb_group)
        self.arm_pub = self.create_publisher(Vector3, '/robot3/cmd_arm', 10)

        self.get_logger().info("Connecting to gripper action server...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper server discovery timed out! Continuing anyway...")

        # Control loop execution running at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Closed-Loop Feedback Grabber Node online. Awaiting MoCap stream...")
    
    def _compute_forward_kinematics(self, joints):
        """
        Compute the forward kinematics for the planar manipulator.
        Returns array [x, z] where x is forward and z is upward.
        """
        theta1, theta2 = joints
        c1 = math.cos(theta1)
        c12 = math.cos(theta1 + theta2)
        s1 = math.sin(theta1)
        s12 = math.sin(theta1 + theta2)
        x = self._a1 * c1 + self._a2 * c12
        z = self._a1 * s1 + self._a2 * s12
        return np.array([x, z])
    
    def robot_pose_callback(self, msg):
        self.robot_pose = np.array([msg.pose.position.x, msg.pose.position.y])
        self.robot_pose_orientation = msg.pose.orientation

    def joint_states_callback(self, msg):
        try:
            idx1 = msg.name.index('robot3/arm_1_joint')
            idx2 = msg.name.index('robot3/arm_2_joint')
            self.joint_positions = np.array([msg.position[idx1], msg.position[idx2]])
            self.joint_velocities = np.array([msg.velocity[idx1], msg.velocity[idx2]])
        except ValueError:
            self.get_logger().error("Arm joint names not found in JointState message!", throttle_duration_sec=5.0)

    def stick_mocap_callback(self, msg):
        if self.robot_pose is None:
            return
        stick_global = np.array([msg.pose.position.x, msg.pose.position.y])
        self.desired_pose = self._convert_to_robot_base_coordinates(stick_global)

    def _convert_to_robot_base_coordinates(self, global_pos):
        translation_error = global_pos - self.robot_pose
        # Extract robot yaw angle from quaternion
        q = self.robot_pose_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Standard 2D coordinate frame rotation matrix projection
        local_x = translation_error[0] * math.cos(yaw) + translation_error[1] * math.sin(yaw)
        local_z = -translation_error[0] * math.sin(yaw) + translation_error[1] * math.cos(yaw)
        
        return np.array([local_x, local_z])

    def control_loop(self):
        # Lockout control loop until telemetry dependencies exist
        if self.joint_positions is None or self.desired_pose is None:
            self.get_logger().warn("Awaiting incoming MoCap stream/telemetry updates...", throttle_duration_sec=2.0)
            return
        
        # --- Phase 0: Ensure Gripper is Completely Open ---
        if self._currentPhase == 0:
            if not self._gripper_action_running:
                self.get_logger().info("Phase 0: Actuating gripper open sequence.")
                self._send_gripper_goal(1) # 1 = OPEN
            return

        pos_error = self._compute_error_vectors()
        pos_error_norm = np.linalg.norm(pos_error)
        
        # --- Phase 1: Close-Loop Proportional Tracking to Target ---
        if self._currentPhase == 1:
            if pos_error_norm < self._error_threshold:
                self.get_logger().info("Phase 1 Complete: Target reached. Braking arm and proceeding to grasp.")
                stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
                self.arm_pub.publish(stop_msg)
                self._currentPhase = 2
            else:
                # Proportional velocity generation
                cmd_x = self._kp * pos_error[0]
                cmd_z = self._kp * pos_error[1]
                
                # Apply velocity clipping bounds to protect workspace edge limits
                cmd_x = np.clip(cmd_x, -self._max_velocity, self._max_velocity)
                cmd_z = np.clip(cmd_z, -self._max_velocity, self._max_velocity)
                
                msg = Vector3(x=float(cmd_x), y=0.0, z=float(cmd_z))
                self.arm_pub.publish(msg)
                
                self.get_logger().info(
                    f"Phase 1 Tracking -> Distance Error: {pos_error_norm:.3f}m | "
                    f"Target Local XZ: [{self.desired_pose[0]:.2f}, {self.desired_pose[1]:.2f}] | "
                    f"Velocity Vector: [{cmd_x:.2f}, {cmd_z:.2f}]", 
                    throttle_duration_sec=1.0
                )

        # --- Phase 2: Close Gripper Around Stick ---
        elif self._currentPhase == 2:
            if not self._gripper_action_running:
                self.get_logger().info("Phase 2: Actuating gripper close sequence.")
                if not self.gripper_client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().error("Gripper server lost! Attempting to trigger goal anyway...")
                self._send_gripper_goal(2) # 2 = CLOSE
                
        # --- Phase 3: Hold Steady Routine Complete ---
        elif self._currentPhase == 3:
            stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
            self.arm_pub.publish(stop_msg)
            self.get_logger().info("Grasp routine finished. Execution standing by.", throttle_duration_sec=5.0)
            
        else:
            self.get_logger().error(f"Invalid execution sequence detected: {self._currentPhase}. Resetting.")
            self._currentPhase = 0

    def _send_gripper_goal(self, state_value):
        self._gripper_action_running = True
        goal_msg = GripperControl.Goal()
        goal_msg.target_state = state_value
        
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._gripper_response_callback)

    def _gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper action goal rejected by hardware driver server.")
            self._gripper_action_running = False
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._gripper_result_callback)

    def _gripper_result_callback(self, future):
        self._gripper_action_running = False  # Clear lock guard
        
        if self._currentPhase == 0:
            self.get_logger().info("Gripper opened successfully. Progressing to Phase 1: MOVE_TO_STICK")
            self._currentPhase = 1
        elif self._currentPhase == 2:
            self.get_logger().info("Gripper clamped successfully. Progressing to Phase 3: DONE")
            self._currentPhase = 3

    def _compute_error_vectors(self):
        current_ee_pose = self._compute_forward_kinematics(self.joint_positions)
        return self.desired_pose - current_ee_pose

def main(args=None):
    rclpy.init(args=args)
    node = StickGrabberNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt caught. Halting manipulator joints...")
    finally:
        try:
            stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
            node.arm_pub.publish(stop_msg)
        except Exception as e:
            print(f"Safety stop command failed to publish: {e}")

        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()