# approach_grab_node.py — offline test runbook (brushbotarium, no internet)

Goal: robot drives to a standoff pose in front of `hockey_sticks_1`, ends
FACE TO FACE, advances straight to grasp distance, closes the gripper.
Phases: `NAVIGATE -> REFINE -> ALIGN -> ADVANCE -> GRASP -> DONE`.

Robot number: session of 2026-07-13 we control **robot4** — substitute your
assigned number everywhere `robot4` / `--robot_id 4` appears.
Placeholders you will fill in during the session:
  **N = ____** (mocap body `dji_robot_N`)   **X = ____** (yaw_offset_deg)
  **Z = ____** (arm height from `/robot4/arm_position`)

--------------------------------------------------------------------------------
## STEP A — before leaving the internet (at home)

1. Files present in `~/hockey` (mounted at `/hockey` inside the container):
   `approach_grab_node.py`, `find_my_body.py`, `run_container.sh`, this file.
2. WSL networking must be MIRRORED for the lab (NAT hides the robot LAN).
   Check from any WSL shell:
   ```bash
   wslinfo --networking-mode          # EXPECT: mirrored
   ```
   If it says `nat`: edit `C:\Users\guanf\.wslconfig` -> `networkingMode=mirrored`,
   then in PowerShell `wsl --shutdown`, reopen WSL, re-check.
   (Mirrored was already enabled on 2026-07-13.)
3. Docker image `dji_robomaster_ros:dev` exists:
   ```bash
   sudo docker images | grep dji_robomaster_ros    # EXPECT: dev tag listed
   ```

--------------------------------------------------------------------------------
## STEP B — arrive at the lab, bring the stack up

1. Join Wi-Fi `brushbotarium` on Windows. "No internet" is NORMAL — all ROS
   traffic is on the LAN.
2. Open a WSL shell. **Wait 30–60 s before touching docker**: snap docker
   crash-loops with a namespace error right after WSL boot and self-heals.
   ```bash
   sudo docker ps        # retry until it answers without error
   ```
3. **Shell 1 — start the container:**
   ```bash
   bash ~/hockey/run_container.sh
   ```
   EXPECT: possibly ">> Registering amd64 (x86_64) emulation...", then
   ">> Starting dji_robomaster_ros:dev ..." and a root prompt INSIDE the
   container. Then, inside:
   ```bash
   source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
   ros2 daemon stop                   # avoid the crowded-network daemon hang
   ```
4. **Every extra shell — attach to the same container:**
   ```bash
   sudo docker exec -it dji_robomaster_ros bash
   source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
   ```
   Rule of thumb: add `--no-daemon` to every `ros2 topic` / `ros2 node` command.

--------------------------------------------------------------------------------
## STEP C — sanity: robot + mocap alive (nothing moves)

```bash
ros2 topic echo /robot4/connected --no-daemon
# EXPECT: data: true    (Ctrl-C)
ros2 topic echo /vrpn_mocap/hockey_sticks_1/pose --field pose.position --no-daemon
# EXPECT: x,y,z streaming at mocap rate    (Ctrl-C)
```
FAIL: no `data: true` -> wrong Wi-Fi / robot off / wrong robot number. STOP.
FAIL: no stick pose -> stick body not in the mocap system; ask the TA.
NOTE: do NOT set `ROS_LOCALHOST_ONLY=1` — the robot driver runs on the
backend PC on the LAN; localhost-only would hide it.

--------------------------------------------------------------------------------
## STEP D — find our mocap body + yaw offset  *** robot spins ~4 s, creeps 20 cm ***

Clear ~1 m around the robot first.
```bash
python3 /hockey/find_my_body.py --ros-args -p robot:=robot4
```
EXPECT: it reports which `/vrpn_mocap/dji_robot_N` rotated with the spin AND
the yaw offset between that body's x-axis and true forward, ending with a
**SUGGESTED FLAGS** line. Write down **N** and **X**.

LESSON (2026-07-13): Vicon streams ALL of dji_robot_1..10 whether or not they
are your robot — a topic existing proves NOTHING, and there is NO reason
N = robot number. Wrong N = controller blind = robot circles at constant
speed. Never skip this step.

--------------------------------------------------------------------------------
## STEP E — calibrate the arm grasp height (nothing moves)

```bash
ros2 topic echo /robot4/arm_position --field point --no-daemon
# note x and z; write down  Z = <the z value>
```
The node will hold the arm at (`arm_x`=0.18, `arm_z`=Z) from the REFINE phase
onward. Keep reach < 0.20 m or `move_arm` gets rejected.

--------------------------------------------------------------------------------
## STEP F — RUN 1: full sequence, SAFE distance  *** ROBOT MOVES ***

Safety: clear ~2 m around robot and stick; hand on Ctrl-C. Ctrl-C publishes a
zero Twist before exiting. Backup e-stop from any attached shell:
```bash
ros2 topic pub --once /robot4/cmd_vel geometry_msgs/msg/Twist "{}"
```

First run stops well short of the stick, so the gripper closes on AIR — this
validates every phase and the gripper action with zero collision risk:
```bash
python3 /hockey/approach_grab_node.py --robot_id 4 --ros-args \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose \
  -p yaw_offset_deg:=X \
  -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
  -p standoff:=0.60 -p approach:=0.50 \
  -p v_max:=0.10 -p w_max:=1.0 \
  -p set_arm:=false
```
EXPECT this exact log sequence:
```
Connecting to gripper action server...
approach_grab_node: /vrpn_mocap/dji_robot_N/pose -> /vrpn_mocap/hockey_sticks_1/pose, ...
Gripper -> OPEN
Gripper open. Approaching...
NAVIGATE: dist 2.xx m, ang_err ...      <- dist counts DOWN, 1 line/s
Point reached (err 0.0xx m). Refining standoff position...
Standoff point reached (err 0.0xx m). Aligning heading...
Face-to-face (err x.x deg). Advancing to 0.5 m...
Grasp distance reached (0.5x m). Closing gripper...
Gripper -> CLOSE
Stick grasped. DONE.
```
And on the floor: robot turns-and-drives (never strafes), pauses at 0.60 m,
rotates in place until its gripper side points at the stick, drives straight
in to 0.50 m, stops, gripper claps shut on air. Robot then holds still.

FAIL modes:
- Stuck at `Waiting for mocap poses...` forever -> wrong N or stick topic.
- Circles at constant speed / drives away -> wrong N or wrong X. Redo STEP D.
- Never moves, gripper opens -> check `/robot4/connected`, v_max not ~0,
  correct `--robot_id` (cmd_vel goes to `/robot4/cmd_vel`).
- `Gripper server not found` at startup -> wrong `--robot_id`; verify with
  `ros2 topic list --no-daemon | grep gripper`.
- Oscillates near the standoff point -> `-p v_max:=0.07` or `-p kp_pos:=0.5`.
- ADVANCE keeps re-aiming/jitters -> stick mocap noisy; slightly raise
  `-p pos_tol:=0.07`.

--------------------------------------------------------------------------------
## STEP G — RUN 2: the real grasp  *** ROBOT TOUCHES THE STICK ***

Reset: Ctrl-C the node, place the robot ~2 m from the stick again.
Now with the arm posture and a real grasp distance:
```bash
python3 /hockey/approach_grab_node.py --robot_id 4 --ros-args \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose \
  -p yaw_offset_deg:=X \
  -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
  -p standoff:=0.50 -p approach:=0.35 \
  -p v_max:=0.10 -p w_max:=1.0 \
  -p set_arm:=true -p arm_x:=0.18 -p arm_z:=Z \
  -p grip_power:=0.5
```
EXPECT: same log sequence as RUN 1, plus the arm drops/extends to the grasp
posture during REFINE/ALIGN (before the final advance), and at the end the
gripper closes ON THE STICK -> `Stick grasped. DONE.` Robot holds position;
there is deliberately NO re-navigation after DONE (the stick now moves with
the robot).

Tune `approach` between runs, ~5 cm at a time:
- Gripper closes SHORT of the stick (air between jaws and stick):
  lower `approach` (0.35 -> 0.30 -> 0.25).
- Body bumps/pushes the stick before GRASP: raise `approach`.
  Geometry: body center->front edge ≈ 0.15 m, arm reach 0.18 m, so the jaws
  sit roughly 0.30 m ahead of the body center — `approach` ≈ 0.30 is the
  theoretical sweet spot IF the stick's mocap centroid is at the shaft.
- Gripper hits the stick too high/low: re-do STEP E, adjust `arm_z`.
- Grip too weak (stick slips when nudged): `-p grip_power:=0.7`.

Verify the grasp: nudge the stick gently — it should move WITH the gripper.

--------------------------------------------------------------------------------
## STEP H — optional variations

- Approach the stick's own +x face (original sim behavior) instead of the
  line-of-sight side — only after confirming which way the stick's Vicon
  x-axis points:
  `-p use_target_yaw:=true -p bearing_deg:=0`
- Moving-target demo: have someone slide the stick BEFORE the GRASP phase —
  the goal recomputes every tick, so the robot re-tracks it. (After DONE it
  intentionally does not.)
- Repeat a run: Ctrl-C, reposition robot, rerun the same command. The gripper
  re-opens automatically at the start of every run.

--------------------------------------------------------------------------------
## Quick troubleshooting (same as TEST_RUNBOOK.md)

- `ros2 topic list` hangs -> use `--no-daemon` (and `ros2 daemon stop` once).
- Everything empty after Wi-Fi switch -> confirm `brushbotarium`; robot driver
  runs on the backend PC on that LAN.
- Docker errors right after boot -> wait 30–60 s (snap crash loop self-heals).
- WSL looks "crashed" after sitting idle -> it's the idle timeout; reopen the
  shell, container may need `bash ~/hockey/run_container.sh` again.
- `move_arm` rejected / arm stuck -> (x,z) outside workspace; re-read
  `/robot4/arm_position`, keep reach < 0.20 m.
- Do NOT set `ROS_LOCALHOST_ONLY=1`.
