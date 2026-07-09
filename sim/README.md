# Home Simulator — RoboMaster EP Hockey Project (ECE 486/687)

A lightweight, **no-ROS** simulator of the Robohub setup so the full mission
can be developed and tested at home on plain Windows/macOS/Linux Python.
Only `numpy` and `matplotlib` are needed.

```
python main.py              # live animation of the full mission
python main.py --speed 3    # animation at 3x real time
python main.py --headless   # fast run: prints phase log + result, saves plots
```

A successful run executes all four project tasks and prints `GOAL!`:

- **T1** navigate to the stick pick-up pose (position *and* orientation)
- **T2** lower the arm, close the gripper, pick up the stick
- **T3** navigate to the pre-shoot pose behind the puck
- **T4** shoot the puck into the goal

Report-quality figures (trajectory, tracking error, unicycle inputs) are saved
to `plots/` after every run — directly usable in the IEEE report.

## Files

| File | Contents |
|------|----------|
| `simulator.py` | World physics: unicycle base (Eq. 1), 2-link arm, gripper, stick, puck friction/impact, goal detection |
| `controller.py` | Approximate linearization (Eqs. 2–3), line-tracking navigation, T1–T4 state machine |
| `main.py` | Runner: animation / headless modes, logging |
| `report_plots.py` | Trajectory, tracking-error, and input plots |

## How this maps to the real ROS2 system

The sim exposes the same interfaces the ROS nodes in this repo already use,
so controller code moves between them almost verbatim:

| Real system (ROS2) | Sim equivalent |
|---|---|
| `/vrpn_mocap/<name>/pose` (PoseStamped) | `sim.get_pose(name)` → `(x, y, yaw)` |
| `/robotX/cmd_vel` (Twist: `linear.x`, `angular.z`) | `sim.set_cmd_vel(v, w)` |
| `/robotX/cmd_arm` (Vector3: `x` fwd, `z` up) | `sim.set_cmd_arm(vx, vz)` |
| `/robotX/gripper` action (GripperControl 1=open, 2=close) | `sim.send_gripper_goal(state)` + `sim.gripper_busy` |
| `/robotX/joint_states` | `sim.arm.q` (same FK as `stick_grabber_node.py`) |
| 10 Hz timer callback | `mission.update()` called at 10 Hz |

Porting to the lab: paste `point_p`, `si_to_uni`, `_track_line`, and the
`HockeyMission` state machine into `move_robot_node.py`, replacing the sim
getters/setters with the mocap subscriptions and publishers already there.
The arm/gripper sequence in T2 is the same logic as `stick_grabber_node.py`
(mocap → robot-frame target → proportional EE velocity → gripper action).

## Control design notes (useful for the report)

- **Approximate linearization** (project Sec. 1.1): the point `p` at offset `l`
  ahead of the unicycle satisfies `p_dot = R(θ) L(l) [v; ω]`, so a
  single-integrator P controller on `p` maps to `[v; ω] = L⁻¹(l) Rᵀ(θ) p_dot`.
  `p` is placed at the stick tip (`l = 0.45 m`) for the shot, per the hint.
- **Pose (not just point) regulation**: to arrive with a desired orientation,
  the controller tracks a straight *approach line* ending at the target and
  pointing along the desired heading. The chassis trails `p` like a trailer,
  so its heading converges to the line direction with spatial constant ≈ `l`.
- **Small `l` while navigating**: heading convergence per meter scales with
  `1/l`, so navigation uses `l_nav = 0.15 m` (aligns in a third of the runway);
  the shot uses the true tip `l = 0.45 m`. This is what makes wall-constrained
  shooting lines feasible.
- **Rotating in place displaces `p`** (it sits `l` ahead), which is why each
  in-place alignment is followed by line tracking rather than plain
  point-to-point navigation.

Caveats vs. reality: the sim is kinematic (no wheel slip/acceleration limits),
mocap is noise-free and always visible, and the stick attaches rigidly on a
successful grasp. Expect to retune gains and tolerances in the Robohub.

## Next fidelity step: the official course simulator

When ready to test actual ROS2 code, use the course simulator
([erablab/multi_robomaster_ros_sim](https://github.com/erablab/multi_robomaster_ros_sim))
inside WSL2 + Docker (see `ece486_ece687_project_README_win.md`):

1. WSL2 is already installed (Ubuntu). Add mirrored networking in
   `%USERPROFILE%\.wslconfig` (`[wsl2]` / `networkingMode=mirrored`).
2. Inside WSL: `sudo snap install docker`
3. `git clone https://github.com/erablab/robomaster_ros.git`, then build the
   image from `robomaster_ros/docker/humble/`
   (`sudo docker build -f Dockerfile -t dji_robomaster_ros:1.0 .`)
4. Clone `multi_robomaster_ros_sim` and start it with its `run.sh`, which
   launches that image; your nodes then talk to simulated `/robotX/...` topics.
