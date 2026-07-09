"""Approximate linearization control + T1-T4 mission state machine.

Implements the method required by the project description (Sec. 1.1):
the point p at the stick tip obeys  p_dot = R(theta) L(l) [v; w]  (Eq. 2),
so a single-integrator controller designed for p_dot maps back to unicycle
inputs via  [v; w] = L^-1(l) R^T(theta) p_dot  (Eq. 3).

Navigation to a pose (position + orientation) uses a pre-approach waypoint
placed behind the target along the desired heading: driving p through it
makes the chassis heading converge to the approach direction, after which a
short in-place rotation removes the residual.

This file is plain Python; the same functions can be pasted into the ROS2
nodes (move_robot_node.py / stick_grabber_node.py) unchanged.
"""
import numpy as np

from simulator import GRIPPER_OPEN, GRIPPER_CLOSE


# ---- approximate linearization (Eqs. 2-3) -----------------------------------

def point_p(pose, l):
    """Position of the control point p, offset l ahead of the unicycle."""
    x, y, th = pose
    return np.array([x + l * np.cos(th), y + l * np.sin(th)])


def si_to_uni(p_dot, theta, l):
    """[v; w] = L^-1(l) R^T(theta) p_dot  -- Eq. (3)."""
    c, s = np.cos(theta), np.sin(theta)
    v = c * p_dot[0] + s * p_dot[1]
    w = (-s * p_dot[0] + c * p_dot[1]) / l
    return v, w


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class HockeyMission:
    """Sequences T1 (nav to stick) -> T2 (pick up) -> T3 (nav to puck)
    -> T4 (shoot into the goal). Call update() at the control rate (10 Hz),
    exactly like the ROS2 timer callback."""

    # gains / limits (same style as the ROS nodes in this repo)
    KP = 1.5                 # proportional gain on p
    K_LAT = 3.0              # lateral gain when tracking an approach line
    KTH = 2.0                # heading gain for in-place alignment
    NAV_SPEED = 0.5          # |p_dot| clamp while navigating (m/s)
    SHOOT_SPEED = 1.6        # |p_dot| along the shooting line (m/s)
    KP_ARM = 1.5             # arm EE proportional gain (stick_grabber_node)
    ARM_MAX = 0.25
    P_TOL = 0.03             # arrival tolerance on p at a final pose (m)
    WP_TOL = 0.08            # looser tolerance at intermediate waypoints (m)
    TH_TOL = 0.05            # heading tolerance (rad)
    ARM_TOL = 0.03           # EE tolerance (m)
    STANDOFF_T1 = 1.3        # approach-line length to the grasp pose (m)
    STANDOFF_T3 = 1.2        # approach-line length to the pre-shoot pose (m)
    PRE_DIST = 0.55          # pre-shoot distance of p behind the puck (m)
    FOLLOW_THROUGH = 0.35    # keep pushing this far past the puck (m)

    def __init__(self, sim):
        self.sim = sim
        self.l = sim.STICK_TIP_OFFSET
        # Smaller offset while navigating: the chassis heading converges to the
        # path direction with spatial constant ~l, so a short l aligns the robot
        # in much less runway. The stick tip (l = 0.45) is used for the shot.
        self.l_nav = 0.15
        self.ctrl_l = self.l_nav        # offset the controller currently uses
        self.phase = 'T1_NAV_WP'
        self.p_des = point_p(sim.get_pose('dji_robot_3'), self.l_nav)

        # T1: park so the arm EE can land on the stick handle, facing it.
        # The approach line ends at the grasp pose and points along the desired
        # final heading, so tracking it aligns the chassis while it arrives.
        sx, sy, syaw = sim.get_pose('hockey_sticks_1')
        self.grasp_heading = wrap(syaw + np.pi)          # face the handle butt
        u = np.array([np.cos(self.grasp_heading), np.sin(self.grasp_heading)])
        ee_fwd = sim.ARM_BASE_FWD + 0.25                 # EE reach when grasping
        grasp_p = np.array([sx, sy]) + (self.l_nav - ee_fwd) * u
        self.t1_line = self._approach_line(grasp_p, u, self.STANDOFF_T1)

        # T3/T4: shooting line from the known puck and goal poses
        px, py, _ = sim.get_pose('puck_1')
        gx, gy, _ = sim.get_pose('goal_1')
        self.puck0 = np.array([px, py])
        self.shoot_dir = np.array([gx - px, gy - py])
        self.shoot_dir /= np.linalg.norm(self.shoot_dir)
        self.shoot_heading = np.arctan2(self.shoot_dir[1], self.shoot_dir[0])
        # end of the nav line places the *tip* at PRE_DIST behind the puck
        preshoot_nav = (self.puck0
                        - (self.PRE_DIST + self.l - self.l_nav) * self.shoot_dir)
        self.t3_line = self._approach_line(preshoot_nav, self.shoot_dir,
                                           self.STANDOFF_T3)

        self.carry_target = np.array([0.20, 0.15])       # EE pose while carrying

    # ---- building blocks ----------------------------------------------------
    def _approach_line(self, end, u, want_len):
        """Approach line ending at `end` along direction `u`. The start point is
        pulled in until it lies inside the field with enough margin for the
        chassis (which trails p by l), even if that shortens the line."""
        lim = self.sim.FIELD - 0.55
        t = want_len
        while t > 0.35 and np.any(np.abs(end - t * u) > lim):
            t -= 0.05
        return {'a': end - t * u, 'dir': u, 'len': t}

    def _nav_to(self, p_target, speed=None):
        """P control on p, mapped to (v, w) via Eq. (3). Returns |error|."""
        self.ctrl_l = self.l_nav
        pose = self.sim.get_pose('dji_robot_3')
        p = point_p(pose, self.l_nav)
        e = p_target - p
        p_dot = self.KP * e
        n = np.linalg.norm(p_dot)
        cap = speed if speed is not None else self.NAV_SPEED
        if n > cap:
            p_dot *= cap / n
        self.sim.set_cmd_vel(*si_to_uni(p_dot, pose[2], self.l_nav))
        self.p_des = p_target
        return np.linalg.norm(e)

    def _track_line(self, line):
        """Track a straight approach line with p: advance along it (slowing down
        near the end) while a lateral P term pulls p onto the line. The chassis
        heading converges to the line direction like a trailer behind p.
        Returns the remaining distance to the end of the line."""
        self.ctrl_l = self.l_nav
        pose = self.sim.get_pose('dji_robot_3')
        p = point_p(pose, self.l_nav)
        rel = p - line['a']
        along = rel @ line['dir']
        lat = rel - along * line['dir']
        remaining = line['len'] - along
        speed = np.clip(self.KP * remaining, 0.08, self.NAV_SPEED)
        p_dot = speed * line['dir'] - self.K_LAT * lat
        self.sim.set_cmd_vel(*si_to_uni(p_dot, pose[2], self.l_nav))
        self.p_des = line['a'] + np.clip(along, 0, line['len']) * line['dir']
        return remaining

    def _align_to(self, heading):
        """Rotate in place to the desired final orientation."""
        pose = self.sim.get_pose('dji_robot_3')
        self.p_des = point_p(pose, self.ctrl_l)
        err = wrap(heading - pose[2])
        self.sim.set_cmd_vel(0.0, np.clip(self.KTH * err, -1.5, 1.5))
        return abs(err)

    def _arm_to(self, target):
        """Proportional EE control, same law as stick_grabber_node phase 1."""
        e = target - self.sim.arm.fk()
        cmd = np.clip(self.KP_ARM * e, -self.ARM_MAX, self.ARM_MAX)
        self.sim.set_cmd_arm(*cmd)
        return np.linalg.norm(e)

    def _grasp_arm_target(self):
        """Stick handle expressed in the robot frame (mocap -> local), the same
        computation as _convert_to_robot_base_coordinates in the ROS node."""
        x, y, th = self.sim.get_pose('dji_robot_3')
        sx, sy, _ = self.sim.get_pose('hockey_sticks_1')
        local_x = (np.array([sx - x, sy - y])
                   @ np.array([np.cos(th), np.sin(th)])) - self.sim.ARM_BASE_FWD
        return np.array([np.clip(local_x, 0.12, 0.34), self.sim.STICK_HANDLE_Z])

    # ---- state machine ------------------------------------------------------
    def update(self):
        sim = self.sim

        if self.phase == 'T1_NAV_WP':
            if self._nav_to(self.t1_line['a']) < self.WP_TOL:
                self.phase = 'T1_APPROACH'

        elif self.phase == 'T1_APPROACH':
            if self._track_line(self.t1_line) < self.P_TOL:
                sim.set_cmd_vel(0.0, 0.0)
                self.phase = 'T1_ALIGN'

        elif self.phase == 'T1_ALIGN':
            if self._align_to(self.grasp_heading) < self.TH_TOL:
                sim.set_cmd_vel(0.0, 0.0)
                self.phase = 'T2_ARM_DOWN'

        elif self.phase == 'T2_ARM_DOWN':
            if self._arm_to(self._grasp_arm_target()) < self.ARM_TOL:
                sim.set_cmd_arm(0.0, 0.0)
                sim.send_gripper_goal(GRIPPER_CLOSE)
                self.phase = 'T2_GRIP'

        elif self.phase == 'T2_GRIP':
            if not sim.gripper_busy:
                if sim.stick_grasped:
                    self.phase = 'T2_ARM_UP'
                else:  # missed: reopen and try the reach again
                    sim.send_gripper_goal(GRIPPER_OPEN)
                    self.phase = 'T2_ARM_DOWN'

        elif self.phase == 'T2_ARM_UP':
            if self._arm_to(self.carry_target) < self.ARM_TOL:
                sim.set_cmd_arm(0.0, 0.0)
                self.phase = 'T3_NAV_WP'

        elif self.phase == 'T3_NAV_WP':
            if self._nav_to(self.t3_line['a']) < self.WP_TOL:
                self.phase = 'T3_APPROACH'

        elif self.phase == 'T3_APPROACH':
            if self._track_line(self.t3_line) < self.P_TOL:
                sim.set_cmd_vel(0.0, 0.0)
                self.phase = 'T3_ALIGN'

        elif self.phase == 'T3_ALIGN':
            if self._align_to(self.shoot_heading) < self.TH_TOL:
                sim.set_cmd_vel(0.0, 0.0)
                self.phase = 'T4_SHOOT'

        elif self.phase == 'T4_SHOOT':
            # drive p (the stick tip now) along the puck->goal line at shooting
            # speed, with a proportional correction of the lateral offset
            self.ctrl_l = self.l
            pose = sim.get_pose('dji_robot_3')
            p = point_p(pose, self.l)
            along = (p - self.puck0) @ self.shoot_dir
            lat = (p - self.puck0) - along * self.shoot_dir
            p_dot = self.SHOOT_SPEED * self.shoot_dir - self.K_LAT * lat
            sim.set_cmd_vel(*si_to_uni(p_dot, pose[2], self.l))
            self.p_des = self.puck0 + max(along, 0.0) * self.shoot_dir
            if along > self.FOLLOW_THROUGH:
                self.phase = 'T4_BRAKE'

        elif self.phase == 'T4_BRAKE':
            sim.set_cmd_vel(0.0, 0.0)
            self.p_des = point_p(sim.get_pose('dji_robot_3'), self.ctrl_l)
            if sim.goal_scored or sim.puck_dead or np.linalg.norm(sim.puck_vel) < 1e-3:
                self.phase = 'DONE'

        elif self.phase == 'DONE':
            sim.set_cmd_vel(0.0, 0.0)

        return self.phase
