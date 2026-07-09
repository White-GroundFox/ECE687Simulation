import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import JointState
from robomaster_msgs.action import GripperControl
import numpy as np
import math
import os
from datetime import datetime

class StickGrabberNode(Node):
    def __init__(self):
        super().__init__('stick_grabber_node')
        
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # --- File Logging Configuration ---
        self.log_filename = "stick_grabber_log_" + str(self.get_clock().now().to_msg().sec) + ".txt"
        with open(self.log_filename, "w") as f:
            f.write(f"--- Debug Logging Session Started: {datetime.now()} ---\n")
            f.write("Timestamp,Phase,Target_Local_X,Target_Local_Z,Error_X,Error_Z,Error_Norm,Joint_1,Joint_2\n")        
        
        self._log_to_file("Stick Grabber Node initialized.")
        
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
        self._currentPhase = 1  # Updated to jump straight to Phase 1 tracking if gripper is open
        self._gripper_action_running = False

        # --- Subscriptions ---
        self._log_to_file("Awaiting MoCap stream and telemetry...")
        self.create_subscription(PoseStamped, '/vrpn_mocap/hockey_sticks_1/pose', self.stick_mocap_callback, mocap_qos)  
        self.create_subscription(JointState, '/robot3/joint_states', self.joint_states_callback, mocap_qos)
        self.create_subscription(PoseStamped, '/vrpn_mocap/dji_robot_3/pose', self.robot_pose_callback, mocap_qos)
        self._log_to_file("Subscriptions to MoCap and JointState topics established.")

        # --- Action Clients & Publishers ---
        self._log_to_file("Initializing gripper action client and arm command publisher...")
        self.cb_group = ReentrantCallbackGroup()
        self.gripper_client = ActionClient(self, GripperControl, '/robot3/gripper', callback_group=self.cb_group)
        self.arm_pub = self.create_publisher(Vector3, '/robot3/cmd_arm', 10)
        self._log_to_file("Arm command publisher established on /robot3/cmd_arm topic.")

        self._log_to_file("Connecting to gripper action server...")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self._log_to_file("[ERROR] Gripper server discovery timed out! Continuing anyway...")
        else:
            self._log_to_file("Gripper action client connected to server successfully.")

        # Control loop execution running at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self._log_to_file("Closed-Loop Feedback Grabber Node online. Awaiting MoCap stream...")
    
    def _log_to_file(self, msg, is_matrix_data=False):
        """Unified logging wrapper that timestamps, saves to disk, and prints to terminal."""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            if is_matrix_data:
                full_line = f"{timestamp},{msg}"
            else:
                full_line = f"[{timestamp}] {msg}"
                # Mirror system/event text back to console out for visibility
                self.get_logger().info(msg)
                
            with open(self.log_filename, "a") as f:
                f.write(full_line + "\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file: {e}")

    def _compute_forward_kinematics(self, joints):
        """
        Compute the forward kinematics for the planar manipulator.
        Assumes independent/absolute servo angles relative to the horizontal frame.
        Returns array [x, z] where x is forward and z is upward.
        """
        theta1, theta2 = joints
        
        # Calculate individual link projections independently
        x = self._a1 * math.cos(theta1) + self._a2 * math.cos(theta2)
        z = self._a1 * math.sin(theta1) + self._a2 * math.sin(theta2)
        
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
        if self.robot_pose is None or self.robot_pose_orientation is None:
            return
        self.desired_pose = self._convert_to_robot_base_coordinates(msg)
    
    def _convert_to_robot_base_coordinates(self, global_msg):
        # 1. Handle horizontal translation error (using global X and Y)
        translation_error_x = global_msg.pose.position.x - self.robot_pose[0]
        translation_error_y = global_msg.pose.position.y - self.robot_pose[1]

        # Extract robot yaw angle from quaternion orientation
        q = self.robot_pose_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 2. Standard 2D rotation matrix projection onto the robot's heading
        local_x = translation_error_x * math.cos(yaw) + translation_error_y * math.sin(yaw)
        # 3. Handle the vertical height target off the ground/base link
        local_z = global_msg.pose.position.z 
        
        return np.array([local_x, local_z])

    def control_loop(self):
        # Lockout control loop until telemetry dependencies exist
        if self.joint_positions is None or self.desired_pose is None:
            self._log_to_file("System Telemetry Blocked: Awaiting stream packets...", is_matrix_data=False)
            return
        
        # --- Phase 0: Ensure Gripper is Completely Open ---
        if self._currentPhase == 0:
            if not self._gripper_action_running:
                self._log_to_file("Phase 0 Execution: Requesting gripper open goal.")
                self._send_gripper_goal(1) # 1 = OPEN
            return

        pos_error = self._compute_error_vectors()
        pos_error_norm = np.linalg.norm(pos_error)
        
        # Stream CSV line formatting directly to file log matrix for Phase 1 profiling
        matrix_str = f"{self._currentPhase},{self.desired_pose[0]:.4f},{self.desired_pose[1]:.4f},{pos_error[0]:.4f},{pos_error[1]:.4f},{pos_error_norm:.4f},{self.joint_positions[0]:.4f},{self.joint_positions[1]:.4f}"
        self._log_to_file(matrix_str, is_matrix_data=True)
        
        if self._currentPhase == 1:
            if pos_error_norm < self._error_threshold:
                self._log_to_file(f"Phase 1 Complete: Target reached (Error: {pos_error_norm:.4f}m). Braking arm joints.")
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

        elif self._currentPhase == 2:
            if not self._gripper_action_running:
                self._log_to_file("Phase 2 Execution: Target secured. Requesting gripper close goal.")
                if not self.gripper_client.wait_for_server(timeout_sec=2.0):
                    self._log_to_file("[ERROR] Gripper server connectivity dropped! Attempting command pulse anyway...")
                self._send_gripper_goal(2) # 2 = CLOSE
                
        # --- Phase 3: Hold Steady Routine Complete ---
        elif self._currentPhase == 3:
            stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
            self.arm_pub.publish(stop_msg)
            
        else:
            self._log_to_file(f"[ERROR] Out-of-bounds execution phase index found: {self._currentPhase}. Hard resetting machine state to 0.")
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
            self._log_to_file("[ERROR] Gripper request handle rejected by low-level firmware architecture.")
            self._gripper_action_running = False
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._gripper_result_callback)

    def _gripper_result_callback(self, future):
        self._gripper_action_running = False  # Clear lock guard
        
        if self._currentPhase == 0:
            self._log_to_file("Gripper opened successfully. Progressing to Phase 1: MOVE_TO_STICK")
            self._currentPhase = 1
        elif self._currentPhase == 2:
            self._log_to_file("Gripper clamped successfully. Progressing to Phase 3: DONE")
            self._currentPhase = 3

    def _compute_error_vectors(self):
        current_ee_pose = self._compute_forward_kinematics(self.joint_positions)
        return self.desired_pose - current_ee_pose

def main(args=None):
    # keep the ROS context alive on Ctrl+C so the safety stop in `finally`
    # can still publish; rclpy's own SIGINT handler would shut it down first
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StickGrabberNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node._log_to_file("Keyboard Interrupt caught. Halting manipulator joints...")
    finally:
        try:
            stop_msg = Vector3(x=0.0, y=0.0, z=0.0)
            node.arm_pub.publish(stop_msg)
            node._log_to_file("Safety stop brake vectors issued to arm publisher.")
        except Exception as e:
            print(f"Safety stop command failed to publish: {e}")

        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()