#!/usr/bin/env python3
"""
goto_goal.py -- drive a RoboMaster EP to a Vicon-tracked object.

UNICYCLE model. The robot hardware is restricted to either PURE straight motion
OR PURE rotation in place -- it may NOT arc (translate and rotate at the same
time). So this uses a TURN-THEN-GO state machine:
    TURN  : rotate in place (v=0, w!=0) until aimed at the goal
    DRIVE : drive straight  (v!=0, w=0) toward the goal
Hysteresis on the heading error (aim-tol / hold-tol) avoids chattering between
the two modes. Pose comes from Vicon via vrpn_mocap.

Run INSIDE the container (folder is mounted at /hockey):
    source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
    ros2 daemon stop                      # avoid the emulation hang
    cd /hockey
    python3 goto_goal.py --robot 4 --robot-obj dji_robot_4 --goal hockey_goal_1

Ctrl-C stops the robot (a zero Twist is published on exit).
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped


def yaw_from_quat(q):
    """Yaw (rotation about world z) from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GoToGoal(Node):
    def __init__(self, a):
        super().__init__('goto_goal')
        self.k_lin = a.k_lin         # proportional gain when driving straight
        self.k_ang = a.k_ang         # proportional gain when rotating in place
        self.v_max = a.v_max
        self.w_max = a.w_max
        self.d_stop = a.standoff     # stop this far from the goal [m]
        self.aim_tol = a.aim_tol     # |heading err| below this -> start driving [rad]
        self.hold_tol = a.hold_tol   # |heading err| above this -> stop and re-aim [rad]
        self.yaw_off = a.yaw_offset  # constant offset if Vicon x-axis != robot forward

        self.robot_pose = None
        self.goal_pose = None
        self.done = False
        self.mode = 'TURN'           # always aim before the first drive

        # vrpn_mocap is high-rate; BEST_EFFORT matches both reliable & best-effort pubs.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.create_subscription(
            PoseStamped, f'/vrpn_mocap/{a.robot_obj}/pose', self.on_robot, qos)
        self.create_subscription(
            PoseStamped, f'/vrpn_mocap/{a.goal}/pose', self.on_goal, qos)
        self.cmd_pub = self.create_publisher(Twist, f'/robot{a.robot}/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.loop)   # 20 Hz
        self.get_logger().info(
            f'goto_goal up. robot=/robot{a.robot}/cmd_vel, '
            f'robot_obj={a.robot_obj}, goal={a.goal}. Waiting for poses...')

    def on_robot(self, msg):
        self.robot_pose = msg

    def on_goal(self, msg):
        self.goal_pose = msg

    def stop(self):
        self.cmd_pub.publish(Twist())   # all-zero

    def loop(self):
        if self.done or self.robot_pose is None or self.goal_pose is None:
            return

        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        th = yaw_from_quat(self.robot_pose.pose.orientation) + self.yaw_off

        gx = self.goal_pose.pose.position.x
        gy = self.goal_pose.pose.position.y

        dist = math.hypot(gx - x, gy - y)
        if dist <= self.d_stop:
            self.get_logger().info(f'Reached goal (dist={dist:.2f} m). Stopping.')
            self.stop()
            self.done = True
            return

        # bearing to the goal and heading error wrapped to (-pi, pi]
        bearing = math.atan2(gy - y, gx - x)
        head_err = math.atan2(math.sin(bearing - th), math.cos(bearing - th))

        # NO ARCING: switch between pure rotation and pure translation only.
        # Hysteresis: start driving once well-aimed (aim_tol); go back to turning
        # only if the heading drifts past the looser hold_tol.
        if self.mode == 'DRIVE' and abs(head_err) > self.hold_tol:
            self.mode = 'TURN'
        elif self.mode == 'TURN' and abs(head_err) <= self.aim_tol:
            self.mode = 'DRIVE'

        cmd = Twist()
        cmd.linear.y = 0.0      # unicycle: NEVER command lateral velocity
        if self.mode == 'TURN':
            # rotate in place: angular only, zero forward speed
            w = max(-self.w_max, min(self.w_max, self.k_ang * head_err))
            cmd.linear.x = 0.0
            cmd.angular.z = w
        else:
            # drive straight: forward only, zero rotation
            v = max(-self.v_max, min(self.v_max, self.k_lin * dist))
            cmd.linear.x = v
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--robot', type=int, default=4, help='robot id for /robotN/cmd_vel')
    p.add_argument('--robot-obj', default='dji_robot_4', help='vrpn_mocap object for the robot')
    p.add_argument('--goal', default='hockey_goal_1', help='vrpn_mocap object for the goal')
    p.add_argument('--k-lin', type=float, default=0.8, help='linear gain (drive straight)')
    p.add_argument('--k-ang', type=float, default=1.5, help='angular gain (rotate in place)')
    p.add_argument('--v-max', type=float, default=0.35, help='max linear speed [m/s]')
    p.add_argument('--w-max', type=float, default=1.5, help='max angular speed [rad/s]')
    p.add_argument('--standoff', type=float, default=0.40, help='stop this far from goal [m]')
    p.add_argument('--aim-tol', type=float, default=0.05,
                   help='heading error to start driving [rad] (~3 deg)')
    p.add_argument('--hold-tol', type=float, default=0.20,
                   help='heading error that forces re-aiming [rad] (~11 deg)')
    p.add_argument('--yaw-offset', type=float, default=0.0, help='constant yaw correction [rad]')
    a = p.parse_args()

    rclpy.init()
    node = GoToGoal(a)
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
