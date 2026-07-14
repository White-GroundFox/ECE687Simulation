#!/usr/bin/env python3
"""
find_my_body.py -- automated STEP-1 "nudge test" + yaw-offset check.

Answers the two questions that must be settled before any autonomous driving:
  1. WHICH /vrpn_mocap/<body>/pose belongs to the robot we command?
     (namespace number != Vicon body number in the lab; getting this wrong
     makes the controller blind -> constant (v,w) -> robot circles in place)
  2. Does that body's yaw point along the robot's forward (+x of cmd_vel)?
     If the Vicon body frame is rotated, approximate linearization drives in
     circles too. Reports the offset to pass as yaw_offset_deg.

THE ROBOT MOVES: ~4 s gentle spin in place, then ~3 s slow forward (~20 cm).
Clear ~0.5 m in front of the robot. Ctrl-C stops it at any time.

Run (inside the container):
  python3 /hockey/find_my_body.py --ros-args -p robot:=robot4
Then copy-paste the whole "NUDGE TEST RESULTS" block.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped

T_BASE = 3.0     # listen only (also lets DDS discovery settle)
T_SPIN = 7.0     # spin in place until here
T_SETTLE = 8.5   # stop + evaluate spin
T_DRIVE = 11.5   # slow forward until here


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class FindMyBody(Node):
    def __init__(self):
        super().__init__('find_my_body')
        robot = self.declare_parameter('robot', 'robot4').value
        self.spin_speed = self.declare_parameter('spin_speed', 0.4).value    # rad/s
        self.drive_speed = self.declare_parameter('drive_speed', 0.06).value # m/s
        self.robot = robot
        self.cmd_pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        # per body: accumulated (unwrapped) yaw so multi-turn spins still count
        self.bodies = {}   # topic -> {'last_raw', 'accum', 'pos', 'n'}
        self.subs = []
        self.snap = {}     # phase label -> {topic: (accum, pos, last_raw)}
        self.winner = None
        self.done = False

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'Nudge test on /{robot}/cmd_vel: 3 s listen, 4 s spin, 3 s forward '
            f'(~20 cm). Watch that it is YOUR robot that moves!')

    # ------------------------------------------------------------------ mocap
    def _subscribe_all(self):
        for topic, types in self.get_topic_names_and_types():
            if (topic.startswith('/vrpn_mocap/') and topic.endswith('/pose')
                    and 'geometry_msgs/msg/PoseStamped' in types):
                self.bodies[topic] = {'last_raw': None, 'accum': 0.0, 'pos': None, 'n': 0}
                self.subs.append(self.create_subscription(
                    PoseStamped, topic, lambda m, t=topic: self._on_pose(m, t), self.qos))
        self.get_logger().info(f'Watching {len(self.bodies)} mocap bodies.')

    def _on_pose(self, msg: PoseStamped, topic: str):
        yaw = yaw_from_quat(msg.pose.orientation)
        b = self.bodies[topic]
        if b['last_raw'] is not None:
            b['accum'] += wrap(yaw - b['last_raw'])
        b['last_raw'] = yaw
        b['pos'] = (msg.pose.position.x, msg.pose.position.y)
        b['n'] += 1

    def _snapshot(self, label):
        self.snap[label] = {k: (v['accum'], v['pos'], v['last_raw'])
                            for k, v in self.bodies.items()}

    # ------------------------------------------------------------ state machine
    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        cmd = Twist()

        if t < T_BASE:
            if not self.subs and t > 1.5:
                self._subscribe_all()
        elif t < T_SPIN:
            if 'spin0' not in self.snap:
                self._snapshot('spin0')
                self.get_logger().info('SPIN phase: robot should rotate in place now.')
            cmd.angular.z = self.spin_speed
        elif t < T_SETTLE:
            if 'spin1' not in self.snap:
                self._snapshot('spin1')
                self._pick_winner()
        elif t < T_DRIVE and self.winner:
            if 'drive0' not in self.snap:
                self._snapshot('drive0')
                self.get_logger().info('DRIVE phase: robot should creep forward ~20 cm.')
            cmd.linear.x = self.drive_speed
        elif not self.done:
            self._snapshot('drive1')
            self._report()
            self.done = True

        self.cmd_pub.publish(cmd)

    def _pick_winner(self):
        expected = self.spin_speed * (T_SPIN - T_BASE)          # rad
        thresh = max(0.15, 0.2 * expected)
        best, best_d = None, 0.0
        for topic in self.bodies:
            a0 = self.snap['spin0'][topic][0]
            a1 = self.snap['spin1'][topic][0]
            d = abs(a1 - a0)
            if d > best_d:
                best, best_d = topic, d
        if best_d > thresh:
            self.winner = best
            self.get_logger().info(
                f'Body that rotated with the spin: {best} '
                f'({math.degrees(best_d):.0f} deg, expected ~{math.degrees(expected):.0f}).')
        else:
            self.get_logger().warn('NO body rotated during the spin. Skipping drive phase.')

    def _report(self):
        lines = ['', '=== NUDGE TEST RESULTS (copy-paste this whole block) ===',
                 f'cmd topic: /{self.robot}/cmd_vel   '
                 f'spin {self.spin_speed} rad/s x {T_SPIN - T_BASE:.0f} s, '
                 f'drive {self.drive_speed} m/s x {T_DRIVE - T_SETTLE:.0f} s',
                 f'{"body":44s} {"msgs":>5s} {"spin dYaw":>10s} {"spin move":>10s}']
        for topic, b in sorted(self.bodies.items()):
            if b['n'] == 0:
                lines.append(f'{topic:44s} {0:5d}  (no data received)')
                continue
            a0, p0, _ = self.snap['spin0'][topic]
            a1, p1, _ = self.snap['spin1'][topic]
            dyaw = math.degrees(a1 - a0)
            move = (math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    if p0 and p1 else float('nan'))
            mark = '   <-- OUR BODY' if topic == self.winner else ''
            lines.append(f'{topic:44s} {b["n"]:5d} {dyaw:9.1f}d {move:9.2f}m{mark}')

        if not self.bodies:
            lines.append('NO /vrpn_mocap/*/pose topics discovered at all -> wrong Wi-Fi /')
            lines.append('networking mode (lab needs mirrored), or vrpn_mocap is down.')
        elif self.winner is None:
            lines.append('-> NO body rotated with the spin. Either the commanded namespace is')
            lines.append(f'   wrong (check: ros2 topic echo /{self.robot}/connected --no-daemon),')
            lines.append('   the robot did not actually move, or its Vicon body is not streaming.')
        elif 'drive0' in self.snap and 'drive1' in self.snap:
            a0, p0, raw0 = self.snap['drive0'][self.winner]
            a1, p1, _ = self.snap['drive1'][self.winner]
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if p0 and p1 else 0.0
            if dist < 0.05:
                lines.append(f'Forward check: only moved {dist:.2f} m -- too little to '
                             'judge the yaw offset; rerun with more room.')
            else:
                bearing = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                yaw_mid = raw0 + (a1 - a0) / 2.0     # body yaw halfway through the drive
                off = wrap(bearing - yaw_mid)
                lines.append(f'Forward check on {self.winner}:')
                lines.append(f'  travelled {dist:.2f} m on world bearing '
                             f'{math.degrees(bearing):.0f} deg, body yaw '
                             f'{math.degrees(wrap(yaw_mid)):.0f} deg '
                             f'-> yaw offset {math.degrees(off):+.0f} deg')
                if abs(off) < math.radians(10):
                    off_flag = 0.0
                    lines.append('  offset negligible: body x-axis = robot forward. GOOD.')
                else:
                    off_flag = round(math.degrees(off), 1)
                    lines.append('  SIGNIFICANT offset: Vicon body frame is rotated vs the')
                    lines.append('  robot forward direction -- pass the flag below.')
                lines.append('SUGGESTED FLAGS for approach_stick_node.py:')
                lines.append(f'  -p robot:={self.robot} '
                             f'-p robot_pose_topic:={self.winner} '
                             f'-p yaw_offset_deg:={off_flag}')
        print('\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FindMyBody()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted: stopping robot.')
    finally:
        for _ in range(3):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
