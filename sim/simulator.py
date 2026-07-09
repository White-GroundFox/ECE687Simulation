"""Kinematic simulator of the Robohub hockey setup (no ROS required).

Emulates the interfaces of the real system so control code ports 1:1 back to
the ROS2 nodes in this repo:

  real system (ROS2)                     sim equivalent
  -------------------------------------  ------------------------------------
  /vrpn_mocap/<name>/pose (PoseStamped)  Sim.get_pose(name) -> (x, y, yaw)
  /robotX/cmd_vel (Twist)                Sim.set_cmd_vel(v, w)
  /robotX/cmd_arm (Vector3)              Sim.set_cmd_arm(vx_fwd, vz_up)
  /robotX/gripper (GripperControl)       Sim.send_gripper_goal(1|2), Sim.gripper_busy
  /robotX/joint_states (JointState)      Sim.arm.q

Frames follow the Robohub convention (Fig. 4b of the project description):
global x/y in the plane, robot heading theta, cmd_vel linear.x forward.
"""
import numpy as np

# robomaster_msgs GripperControl.target_state values
GRIPPER_PAUSE, GRIPPER_OPEN, GRIPPER_CLOSE = 0, 1, 2


class ArmSim:
    """2-link planar arm with absolute joint angles.

    Uses the same forward kinematics as stick_grabber_node.py
    (x = a1*cos(q1) + a2*cos(q2), z = a1*sin(q1) + a2*sin(q2)) and accepts
    end-effector velocity commands like /robotX/cmd_arm (x forward, z up).
    """
    A1, A2 = 0.22, 0.15

    def __init__(self, q1=1.25, q2=-0.4):
        self.q = np.array([q1, q2], dtype=float)
        self.cmd = np.zeros(2)  # commanded EE velocity [x_fwd, z_up] (m/s)

    def fk(self):
        return np.array([
            self.A1 * np.cos(self.q[0]) + self.A2 * np.cos(self.q[1]),
            self.A1 * np.sin(self.q[0]) + self.A2 * np.sin(self.q[1]),
        ])

    def step(self, dt):
        J = np.array([
            [-self.A1 * np.sin(self.q[0]), -self.A2 * np.sin(self.q[1])],
            [ self.A1 * np.cos(self.q[0]),  self.A2 * np.cos(self.q[1])],
        ])
        # damped least squares keeps the arm well-behaved near singularities
        qdot = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(2), self.cmd)
        self.q += np.clip(qdot, -2.0, 2.0) * dt


class Sim:
    """World: one RoboMaster EP (unicycle), a stick, a puck, and a goal."""

    # geometry (m)
    FIELD = 2.5                # half side of the square field
    ARM_BASE_FWD = 0.10        # arm base ahead of robot center
    STICK_TIP_OFFSET = 0.45    # robot center -> stick tip when carried (= l)
    STICK_HANDLE_Z = 0.05      # grasp height of the stick handle
    GRASP_XY_TOL = 0.12        # EE-to-handle tolerance for a successful grasp
    GRASP_Z_TOL = 0.08
    PUCK_R = 0.03
    BLADE_R = 0.05             # effective stick blade radius
    GOAL_WIDTH = 0.60

    # dynamics
    MAX_V, MAX_W = 3.5, 10.0   # hardware limits of the RoboMaster EP
    PUCK_FRICTION = 0.30       # deceleration (m/s^2) of a sliding puck
    RESTITUTION = 0.05         # extra speed transferred on stick impact
    GRIPPER_TIME = 0.8         # gripper action duration (s)

    def __init__(self,
                 robot_start=(0.0, -2.0, np.pi / 2),
                 stick_pose=(1.7, -1.3, 3 * np.pi / 4),
                 puck_pos=(0.3, 0.5),
                 goal_pose=(-1.9, -0.9, 0.0)):
        self.t = 0.0
        self.robot = np.array(robot_start, dtype=float)      # x, y, theta
        self.cmd_v, self.cmd_w = 0.0, 0.0
        self.arm = ArmSim()

        self.stick = np.array(stick_pose, dtype=float)       # x, y, yaw
        self.stick_grasped = False

        self.puck = np.array(puck_pos, dtype=float)
        self.puck_vel = np.zeros(2)

        self.goal = np.array(goal_pose, dtype=float)         # x, y, facing yaw
        self.goal_scored = False
        self.puck_dead = False   # puck stopped against a wall (missed shot)

        self.gripper_state = GRIPPER_OPEN
        self.gripper_busy = False
        self._gripper_target = GRIPPER_OPEN
        self._gripper_timer = 0.0

    # ---- ROS-like interface -------------------------------------------------
    def get_pose(self, name):
        """Mocap emulation: 'dji_robot_3', 'hockey_sticks_1', 'puck_1', 'goal_1'."""
        if name.startswith('dji_robot'):
            return tuple(self.robot)
        if name.startswith('hockey_sticks'):
            if self.stick_grasped:
                tip = self.stick_tip()
                return (tip[0], tip[1], self.robot[2])
            return tuple(self.stick)
        if name.startswith('puck'):
            return (self.puck[0], self.puck[1], 0.0)
        if name.startswith('goal'):
            return tuple(self.goal)
        raise KeyError(name)

    def set_cmd_vel(self, v, w):
        self.cmd_v = float(np.clip(v, -self.MAX_V, self.MAX_V))
        self.cmd_w = float(np.clip(w, -self.MAX_W, self.MAX_W))

    def set_cmd_arm(self, vx, vz):
        self.arm.cmd = np.array([vx, vz], dtype=float)

    def send_gripper_goal(self, target_state):
        """Async like the real action: sets gripper_busy, resolves after a delay."""
        if self.gripper_busy:
            return False
        self._gripper_target = target_state
        self._gripper_timer = self.GRIPPER_TIME
        self.gripper_busy = True
        return True

    # ---- derived quantities -------------------------------------------------
    def heading(self):
        th = self.robot[2]
        return np.array([np.cos(th), np.sin(th)])

    def stick_tip(self):
        """The point p of the approximate linearization = tip of the stick."""
        return self.robot[:2] + self.STICK_TIP_OFFSET * self.heading()

    def stick_tip_vel(self):
        """p_dot = R(theta) L(l) [v; w]  (Eq. (2) of the project description)."""
        th, l = self.robot[2], self.STICK_TIP_OFFSET
        return np.array([
            self.cmd_v * np.cos(th) - l * self.cmd_w * np.sin(th),
            self.cmd_v * np.sin(th) + l * self.cmd_w * np.cos(th),
        ])

    def ee_world(self):
        """Arm end-effector position: (x, y) in the plane and height z."""
        ee = self.arm.fk()
        xy = self.robot[:2] + (self.ARM_BASE_FWD + ee[0]) * self.heading()
        return xy, ee[1]

    # ---- physics ------------------------------------------------------------
    def step(self, dt):
        self.t += dt

        # unicycle base (Eq. (1))
        x, y, th = self.robot
        self.robot[0] = x + self.cmd_v * np.cos(th) * dt
        self.robot[1] = y + self.cmd_v * np.sin(th) * dt
        self.robot[2] = th + self.cmd_w * dt
        self.robot[:2] = np.clip(self.robot[:2], -self.FIELD + 0.2, self.FIELD - 0.2)

        self.arm.step(dt)
        self._step_gripper(dt)
        self._step_puck(dt)

    def _step_gripper(self, dt):
        if not self.gripper_busy:
            return
        self._gripper_timer -= dt
        if self._gripper_timer > 0:
            return
        self.gripper_busy = False
        self.gripper_state = self._gripper_target
        if self._gripper_target == GRIPPER_CLOSE and not self.stick_grasped:
            ee_xy, ee_z = self.ee_world()
            near_xy = np.linalg.norm(ee_xy - self.stick[:2]) < self.GRASP_XY_TOL
            near_z = abs(ee_z - self.STICK_HANDLE_Z) < self.GRASP_Z_TOL
            if near_xy and near_z:
                self.stick_grasped = True
        elif self._gripper_target == GRIPPER_OPEN:
            self.stick_grasped = False

    def _step_puck(self, dt):
        prev = self.puck.copy()

        # stick blade -> puck impact: a flat blade launches the puck along the
        # blade's velocity direction (not center-to-center like two balls)
        if self.stick_grasped:
            tip = self.stick_tip()
            d = self.puck - tip
            dist = np.linalg.norm(d)
            r = self.PUCK_R + self.BLADE_R
            if 1e-9 < dist < r:
                n = d / dist
                v_tip = self.stick_tip_vel()
                if (v_tip - self.puck_vel) @ n > 0:  # blade closing on the puck
                    tip_speed = np.linalg.norm(v_tip)
                    if tip_speed > 0.05:
                        self.puck_vel = (1 + self.RESTITUTION) * v_tip
                    else:
                        self.puck_vel += ((v_tip - self.puck_vel) @ n) * n
                self.puck = tip + r * n  # de-penetrate

        # sliding friction
        speed = np.linalg.norm(self.puck_vel)
        if speed > 1e-6:
            drop = min(speed, self.PUCK_FRICTION * dt)
            self.puck_vel -= drop * self.puck_vel / speed
        self.puck += self.puck_vel * dt

        self._check_goal(prev)

        # walls stop the puck dead (missed shot)
        if np.any(np.abs(self.puck) > self.FIELD - self.PUCK_R) and not self.goal_scored:
            self.puck = np.clip(self.puck, -self.FIELD + self.PUCK_R, self.FIELD - self.PUCK_R)
            if np.linalg.norm(self.puck_vel) > 1e-6:
                self.puck_vel[:] = 0.0
                self.puck_dead = True

    def _check_goal(self, prev):
        if self.goal_scored:
            return
        gx, gy, gth = self.goal
        n = np.array([np.cos(gth), np.sin(gth)])    # mouth normal, into the field
        t = np.array([-np.sin(gth), np.cos(gth)])   # along the goal line
        s_prev = (prev - self.goal[:2]) @ n
        s_now = (self.puck - self.goal[:2]) @ n
        lateral = abs((self.puck - self.goal[:2]) @ t)
        if s_prev > 0 >= s_now and lateral < self.GOAL_WIDTH / 2:
            self.goal_scored = True
            self.puck_vel *= 0.2  # the net catches the puck

    def goal_posts(self):
        gx, gy, gth = self.goal
        t = np.array([-np.sin(gth), np.cos(gth)])
        c = self.goal[:2]
        return c + t * self.GOAL_WIDTH / 2, c - t * self.GOAL_WIDTH / 2
