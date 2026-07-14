# Hockey robot — offline test runbook

Everything here runs on the robot Wi-Fi (`brushbotarium`, no internet). All ROS
traffic is on the LAN, so no internet is needed. Robot is controlled as
`robot3`. Files are mounted at `/hockey` inside the container.

Handy aliases for this session (paste once per shell):
```bash
src() { source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash; }
```

--------------------------------------------------------------------------------
## Shell setup
Open shells as needed. Shell 1 = launch the container; extra shells attach to it.

**Shell 1 (start container):**
```bash
bash ~/hockey/run_container.sh          # re-registers amd64 emulation + runs container
# now inside the container:
source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
ros2 daemon stop                        # avoid the crowded-network daemon hang
```
**Any extra shell (attach to same container):**
```bash
sudo docker exec -it dji_robomaster_ros bash
source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
```
Rule of thumb: add `--no-daemon` to every `ros2 topic`/`node` command.

--------------------------------------------------------------------------------
## STEP 0 — confirm robot + mocap are alive
```bash
ros2 topic echo /robot3/connected --no-daemon            # EXPECT: data: true   (Ctrl-C)
ros2 topic echo /vrpn_mocap/hockey_sticks_1/pose --field pose.position --no-daemon
                                                         # EXPECT: x,y,z streaming (Ctrl-C)
```
FAIL: no `data: true` -> wrong Wi-Fi / robot off / wrong namespace. Stop here.

--------------------------------------------------------------------------------
## STEP 1 — nudge test: which dji_robot_N is our robot?
Mocap tracks dji_robot_1/2/10 but NOT dji_robot_3. Find which one is ours.

**Shell A (watch a candidate — start with 1):**
```bash
ros2 topic echo /vrpn_mocap/dji_robot_1/pose --field pose.orientation --no-daemon
```
**Shell B (spin robot3 gently, then STOP):**
```bash
ros2 topic pub -r 10 /robot3/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"
#   watch Shell A for ~3 s, then Ctrl-C here, then STOP the robot:
ros2 topic pub --once /robot3/cmd_vel geometry_msgs/msg/Twist "{}"
```
- Numbers in Shell A **change while spinning** -> THAT is our robot. Note N.
- No change -> Ctrl-C Shell A, retry with `/vrpn_mocap/dji_robot_2/...`, then `_10`.
- None of the three change -> our robot has no Vicon body; ask the TA.

Write it down:  **N = ____**  (used everywhere below as dji_robot_N)

--------------------------------------------------------------------------------
## STEP 2 — transform test (ee_stick_to_arm.py), reporting only, arm NOT moving
```bash
python3 /hockey/ee_stick_to_arm.py --robot_id 3 --ros-args \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose \
  -p send_arm:=false
```
EXPECT, ~5 Hz:
```
stick in arm frame: x=2.xx y=-0.xx z=-0.0x (reach=2.xx m)
  -> OUT OF ARM REACH: chassis must drive closer
```
- `x` ≈ horizontal distance robot->stick (a couple metres now). GOOD.
- If it only prints "waiting for robot pose + stick pose..." -> wrong N in
  robot_pose_topic, or that body isn't publishing. Fix N.
Ctrl-C to stop. Nothing moves in this step (safe).

--------------------------------------------------------------------------------
## STEP 3 — chassis drive (move_robot_node.py)  *** ROBOT WILL MOVE ***
SAFETY: clear ~2 m around the robot; keep a hand on Ctrl-C; start SLOW.
On Ctrl-C the node publishes a zero Twist to stop the robot.

**Shell A (optional monitor):** rerun STEP 2 command in parallel to watch
`reach` shrink toward ~0.18 and `y` toward 0.

**Shell B (drive):**
```bash
python3 /hockey/move_robot_node.py --ros-args \
  -p robot:=robot3 \
  -p cmd_vel_topic:=/robot3/cmd_vel \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose \
  -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
  -p l:=0.18 -p v_max:=0.10 -p w_max:=1.0
```
EXPECT:
- Robot first TURNS to face the stick, then DRIVES toward it (forward + turning
  only; it never strafes sideways — that's the unicycle model).
- It stops when the control point p (0.18 m ahead of the robot) reaches the
  stick, i.e. the robot body halts ~0.18 m short of the stick center.
- Shell A monitor should reach `-> reachable & aligned.`
FAIL modes:
- Robot doesn't move: check `/robot3/connected` true; v_max not ~0; correct
  cmd_vel topic.
- Drives the wrong way / spins: wrong N (robot pose from a different body) ->
  re-check STEP 1.
- Overshoots/oscillates: lower v_max to 0.07, or Kp to 0.7.

--------------------------------------------------------------------------------
## STEP 4 — grasp (only after STEP 3 shows reachable & aligned)
Calibrate arm height first (read a valid EE position):
```bash
ros2 topic echo /robot3/arm_position --field point --no-daemon    # note x,z
```
Then either:
(a) command the arm to the stick via the transform (arm now reachable):
```bash
python3 /hockey/ee_stick_to_arm.py --robot_id 3 --ros-args \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose -p send_arm:=true
```
EXPECT: "-> reachable & aligned." then "move_arm -> x=.., z=.." then "finished".
(b) or run the dedicated grabber (open -> set height -> close gripper):
```bash
python3 /hockey/stick_grabber_node.py --ros-args \
  -p robot:=robot3 -p arm_x:=0.18 -p arm_z:=<z from echo>
```
EXPECT: gripper OPEN -> arm to grasp posture -> gripper CLOSE -> "DONE".

--------------------------------------------------------------------------------
## STEP 3b — approach + FACE-TO-FACE (approach_stick_node.py) *** ROBOT MOVES ***
Port of the sim-tested ApproachRobotNode (ECE687Simulation repo): drives to a
standoff distance from the stick, then rotates in place to face it, then HOLDs
(re-navigates if the stick moves). Session of 2026-07-13: we control **robot4**.
Prereqs: STEP 0 with `/robot4/connected`, and the AUTOMATED nudge test
(replaces manual STEP 1 — robot spins 4 s then creeps forward ~20 cm):
```bash
python3 /hockey/find_my_body.py --ros-args -p robot:=robot4
```
It prints which /vrpn_mocap body rotated with the spin AND the yaw offset
between that body's x-axis and the robot's true forward, ending in a
SUGGESTED FLAGS line — paste those flags into the command below.
LESSON (2026-07-13): we assumed dji_robot_1 <-> our robot; it was NOT — the
controller was blind (pos_err frozen) and the robot circled in place at
constant (v,w). Vicon streams ALL of dji_robot_1..10, so a topic existing
proves nothing. Stick topic is `hockey_sticks_1` (plural, verified).

```bash
python3 /hockey/approach_stick_node.py --ros-args \
  -p robot:=robot4 \
  -p robot_pose_topic:=/vrpn_mocap/dji_robot_N/pose \
  -p yaw_offset_deg:=X \
  -p target_topic:=/vrpn_mocap/hockey_sticks_1/pose \
  -p standoff:=0.50 -p v_max:=0.10 -p w_max:=1.0
```
(N and X come from the find_my_body.py SUGGESTED FLAGS line. There is NO
reason N = 4: on 2026-07-13 mocap streamed ALL of dji_robot_1..10.)
EXPECT: log `NAVIGATE: pos_err ...` counting down -> "Position reached" ->
short in-place rotation -> "Face-to-face ... Holding." Body center stops
0.50 m from the stick center, gripper side pointing at it. Ctrl-C publishes a
zero Twist (and the 500 ms watchdog backstops it).

Options:
- Closer: `-p standoff:=0.35` (keep >= 0.30 so the body never touches).
- `-p use_target_yaw:=true -p bearing_deg:=0` = original sim behavior: park in
  front of the STICK's +x face instead of on our own line of sight. Only after
  checking which way the stick's Vicon x-axis actually points.
- Oscillates near the goal: lower v_max to 0.07 or kp_pos to 0.5.
- Never leaves "Waiting for mocap poses...": wrong N or stick topic name.

--------------------------------------------------------------------------------
## Quick troubleshooting
- `ros2 topic list` hangs -> use `--no-daemon` (and `ros2 daemon stop` once).
- Everything empty after Wi-Fi switch -> confirm joined `brushbotarium`; the
  robot driver runs on the backend PC on that LAN.
- `move_arm` rejected / arm stuck at a limit -> target (x,z) outside workspace;
  re-read `/robot3/arm_position` for valid numbers, keep reach < 0.20 m.
- Do NOT set ROS_LOCALHOST_ONLY=1 here — the robot driver is on the network,
  localhost-only would hide it.
