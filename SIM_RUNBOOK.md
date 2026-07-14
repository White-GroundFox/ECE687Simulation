# Hockey robot — HOME SIMULATION runbook (navigation only)

Runs entirely at home: no robot, no `brushbotarium` Wi-Fi. The official course
simulator (`erablab/multi_robomaster_ros_sim`, verbatim copy in
`/hockey/sim/simulator.py`) replaces the robot + Vicon:

| Lab                                   | Sim (same topic names!)              |
|---------------------------------------|--------------------------------------|
| robot listens on `/robot3/cmd_vel`    | sim listens on `/robot3/cmd_vel`     |
| Vicon pubs `/vrpn_mocap/dji_robot_N/pose` | sim pubs poses for robots 3 AND 6 |
| Vicon pubs stick pose                 | **missing** -> we fake it with `ros2 topic pub` (TEST A only) |

Bonus vs the lab: in sim, robot ID and mocap body ID always match
(`robot3` <-> `dji_robot_3`), so no STEP-1 nudge test needed.
The sim also reproduces the 500 ms cmd_vel watchdog: stop publishing and the
robot stops, like the real one.

Same shell aliases as the lab runbook:
```bash
src() { source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash; }
```

--------------------------------------------------------------------------------
## NETWORKING — read this first when switching home <-> lab
WSL networking mode decides whether ROS2 works AT ALL (`C:\Users\guanf\.wslconfig`):

| Where | Mode | .wslconfig |
|-------|------|-----------|
| HOME (sim) | NAT (default) | `networkingMode=mirrored` COMMENTED OUT |
| LAB (robots) | mirrored | `networkingMode=mirrored` ACTIVE |

Why: mirrored mode breaks UDP multicast on loopback -> DDS discovery fails ->
every node runs fine but NOBODY sees anyone else's topics (`topic echo` says
"does not appear to be published yet" while the publisher is demonstrably
publishing). Diagnosed 2026-07-08; also the cause of `ros2 daemon` TCP
timeouts at home.

After editing .wslconfig: `wsl --shutdown` in PowerShell, reopen WSL shells.
(run_container.sh re-registers amd64 emulation automatically on next start.)

--------------------------------------------------------------------------------
## STEP 0 — one-time: check GUI deps in the image
The sim draws with matplotlib + Qt5. These live INSIDE the container image,
not in WSL — so first start the container (`bash ~/hockey/run_container.sh`).

How to tell which shell you're in:
- WSL:       `guanf@SurfaceToMoon:~$`   <- WRONG place for this step
- container: `root@SurfaceToMoon:/#`    <- run the commands here

```bash
python3 -c "import matplotlib, PyQt5; print('OK')"
```
If it fails (home Wi-Fi = internet available; no sudo — you are root here;
slow under emulation, minutes are normal):
```bash
apt-get update && apt-get install -y python3-matplotlib python3-pyqt5
```
The container runs with `--rm`, so installs vanish on exit. To keep them,
**while the container is still running**, from a WSL shell:
```bash
sudo docker commit dji_robomaster_ros dji_robomaster_ros:dev
```

--------------------------------------------------------------------------------
## Shell 1 — container + simulator
```bash
bash ~/hockey/run_container.sh
# inside the container:
source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
python3 /hockey/sim/simulator.py
```
Do NOT run `ros2 daemon stop` — under mirrored networking it hangs ~2 min then
TimeoutErrors. Python nodes never use the daemon; only `ros2 topic ...` CLI
commands do, and `--no-daemon` on those is enough.

EXPECT: log line `Simulator started for robots: [3, 6]` and a matplotlib window
(WSLg) with a 4x4 m arena and TWO black robots: 3 at a random spot, 6 fixed at
the center. First paint can take ~10-20 s under emulation — be patient once.
(Our copy is patched vs the original: ROBOT_IDS trimmed to [3, 6] — robot 6 is
the TEST B approach target, pinned to (0, 0, theta=1.2) by the line right after
the random-init loop; edit that line to move it — and GUI redraw decimated to
~3.3 Hz, because a full 33 Hz redraw freezes Qt under QEMU. Physics + pose
publishing still run at the original 33 Hz.)

FAIL modes:
- `qt.qpa.plugin: could not connect to display` -> WSLg mounts missing; make
  sure you launched via `run_container.sh` (it sets `DISPLAY=:0` and mounts
  `/mnt/wslg`). Test X with: `ls /tmp/.X11-unix` (must show `X0`).
- No module PyQt5/matplotlib -> STEP 0.
- Window appears in taskbar but never paints: most likely matplotlib is
  building its font cache — minutes under emulation. It is now persisted in
  `/hockey/.mplcache` (via MPLCONFIGDIR in run_container.sh), so this cost is
  paid ONCE ever. Check progress from another shell: `top` -> python3 near
  100% CPU = working, wait; near 0% = actually stuck, dig deeper. Build is
  done when `fontlist-v*.json` appears in `/hockey/.mplcache`. Note: poses
  only start publishing AFTER the first draw completes, so an empty
  `topic echo` during the build is normal, not a bug.
- Window still sluggish: raise `PLOT_EVERY` in `/hockey/sim/simulator.py`
  (10 -> 20 gives ~1.6 Hz redraw). Control behavior is unaffected.
- GUI totally hopeless? You can still test: the sim's ROS side runs fine
  without the window painting — verify motion numerically with
  `ros2 topic echo /vrpn_mocap/dji_robot_3/pose --field pose.position --no-daemon`.
- Robots drawn off-view (the sim has a known xlim/ylim typo): use the
  magnifier/pan buttons in the plot toolbar.

--------------------------------------------------------------------------------
## Shell 2 — verify topics, then fake the stick (stick = TEST A only)
Attach: `sudo docker exec -it dji_robomaster_ros bash` + `src`.
```bash
ros2 topic list --no-daemon | grep vrpn        # EXPECT dji_robot_3 AND dji_robot_6
ros2 topic echo /vrpn_mocap/dji_robot_3/pose --field pose.position --no-daemon
                                               # EXPECT x,y streaming (Ctrl-C)
```
TEST A only — publish the fake stick (= navigation target) and LEAVE IT RUNNING
(TEST B needs no stick; robot 6 is the target, skip straight to its section).
Pick any point inside the arena ([-2,2] x [-2,2]), e.g. (1, 1):
```bash
ros2 topic pub -r 10 /vrpn_mocap/hockey_sticks_1/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"
```

--------------------------------------------------------------------------------
## Shell 3 — TEST A: move_robot_node.py drives to the faked stick
Attach: `sudo docker exec -it dji_robomaster_ros bash` + `src`.
```bash
python3 /hockey/move_robot_node.py --ros-args \
  -p robot:=robot3 \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_3/pose \
  -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
  -p l:=0.20 -p v_max:=0.25
```
EXPECT (watch the sim window): robot 3 turns toward (1,1), drives forward-only
(never strafes), and halts with the gripper notch ~on the target (body center
stops ~l = 0.20 m short). Kill the node (Ctrl-C) -> robot freezes ≤ 0.5 s later
(watchdog working).

Then iterate freely — this is the whole point of sim:
- Move the goal: Ctrl-C the Shell-2 pub, restart with new x/y; robot re-chases.
- T1 "desired orientation": add
  `-p align_final_yaw:=true -p goal_yaw:=1.57` -> after arrival it rotates in
  place to face +y.
- Tune `Kp`, `v_max`, `l`, `pos_tol` at sim speeds (v_max 0.25+) before ever
  trying them on hardware.

FAIL modes:
- Robot ignores commands: `ros2 topic echo /robot3/cmd_vel --no-daemon` while
  the controller runs — if Twists stream but nothing moves, sim/controller are
  in different ROS domains (both shells must be in the SAME container: plain
  `docker exec`, no extra env).
- Node waits forever: target pub in Shell 2 died, or typo'd topic name.
- Spins/overshoots around the goal: same tuning fixes as the lab runbook
  (lower `v_max` / `Kp`); note oscillation shows up here with IDEAL poses —
  if it oscillates in sim it will be worse on hardware.

--------------------------------------------------------------------------------
## TEST B — approach_robot_node.py: robot 3 approaches robot 6
Shell 1 exactly as above. NO fake stick — robot 6 IS the target (pinned at
(0, 0, theta=1.2) in `sim/simulator.py`; robot 6's theta decides where robot 3
parks, because the approach bearing is measured in ROBOT 6's frame).

Shell 2: attach + `src`, confirm BOTH poses stream:
```bash
ros2 topic list --no-daemon | grep vrpn   # need dji_robot_3 AND dji_robot_6
```

Shell 3: attach + `src`. All mission config is hardcoded at the top of the
file — no `--ros-args`:
```bash
python3 /hockey/approach_robot_node.py
```

Phases: NAVIGATE (approx.-linearization drive to the standoff point) -> REFINE
(turn-in-place + straight creep; kills the ~l lateral residual the p-controller
leaves, else the final pose sits 5-15 deg off robot 6's nose axis) -> ALIGN
(rotate to face robot 6) -> ADVANCE (straight in) -> HOLD (zero cmd).

EXPECT this log, then SILENCE:
```
Point reached (err 0.050 m). Refining standoff position...
Standoff point reached (err 0.049 m). Aligning heading...
Aligned (err -2.8 deg). Advancing to 0.5 m...
Approach distance reached (0.55 m). Holding pose.
```
`Target moved. Re-navigating...` may ONLY appear if robot 6 actually moves.
If it repeats forever, the HOLD hysteresis broke: the `_replan_*` tolerances
must stay LOOSER than the residuals `_pos_tol`/`_ang_tol` leave behind
(diagnosed 2026-07-13 — HOLD once watched the goal-center error, which can
NEVER settle below ~l + tol, so it re-triggered in an infinite micro-adjust
loop).

Knobs (all at the top of approach_robot_node.py, restart node after editing):
- `STANDOFF_DISTANCE` (1.0 m)  waypoint in front of robot 6
- `APPROACH_BEARING_DEG` (0)   approach side in robot 6's frame; 180 = behind
- `APPROACH_DISTANCE` (0.5 m)  final spacing; keep > ~0.35 (bodies are ~0.3 m)
- `_pos_tol` (0.05 m)  stop tolerance: ADVANCE halts at +tol (hence "0.55"),
  final skew ~ atan(tol/standoff) ~ 3 deg. 0.02 fine in sim; keep 0.05 for lab.
- `_replan_dist_tol` / `_replan_ang_tol`  HOLD re-engage hysteresis, see above.

Test the re-engage: nudge robot 6 from another shell — robot 3 must back out
to the standoff point, re-align, and come in again:
```bash
ros2 topic pub -r 5 /robot6/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
(Ctrl-C the pub; robot 6 freezes via the same 500 ms watchdog.)

--------------------------------------------------------------------------------
## Sim vs lab — what this does and does not validate
Validates: approximate-linearization math, topic wiring, QoS, waypoint logic,
arrival tolerances, final-heading alignment, watchdog handling.
Does NOT validate: mecanum slip/acceleration limits, Vicon noise/dropout,
arm/gripper (`cmd_arm`, GripperControl action — no server in sim; those nodes
will hang at wait_for_server), stick/puck contact. Re-tune gains ~30% softer
when moving to hardware.

Later extension plan (when navigation is solid): mock GripperControl action
server + stick/puck poses published by an extended sim node.
