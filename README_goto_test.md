# goto_goal — robot navigation test (ECE 486/687 hockey)

Drives a RoboMaster EP to a Vicon-tracked object. Pose comes from Vicon via
`vrpn_mocap`. Unicycle model — only `linear.x` and `angular.z` are commanded
(`linear.y = 0`).

## Motion model: TURN-THEN-GO (no arcing)
The robot hardware is restricted to **either pure straight motion OR pure
rotation in place — never both at once** (arcing/steering-while-driving is
banned). So the controller uses a two-mode state machine; at any instant only
ONE of `linear.x` / `angular.z` is nonzero:

- **TURN**  — rotate in place toward the goal bearing (`v = 0`, `w = k_ang · heading_err`)
- **DRIVE** — drive straight at the goal (`v = k_lin · dist`, `w = 0`)

It starts in TURN (aims before moving), switches to DRIVE once aimed within
`--aim-tol`, and switches back to TURN only if heading drifts past `--hold-tol`.
The two thresholds are hysteresis — they stop it flip-flopping near the boundary.

> Note: this is a switching unicycle controller, NOT the approximate
> (point-offset) feedback-linearization method, because that law always commands
> `v` and `w` simultaneously (arcs), which the hardware forbids.

## Files
- `goto_goal.py`     — the controller
- `run_container.sh` — launches the ROS dev container (see platform note below)

## Prerequisites
- The `dji_robomaster_ros:dev` Docker image (build from the course Dockerfile, or
  load the shared `dji_ros.tar.gz` via `docker load < dji_ros.tar.gz`).
- On the **brushbotarium** Wi-Fi (pw: `brushbotarium`) so the container can reach
  the robots and the mocap stream.
- Robot powered on and tracked by Vicon (has a rigid body in the mocap system).

## Platform note
`run_container.sh` is written for an **ARM64** host (it emulates x86_64 with
`--platform linux/amd64` via QEMU — slow but required on ARM).
On a normal **x86_64** laptop you do NOT need emulation; launch with:

    sudo docker run -it --rm --network=host --pid=host --ipc=host \
      -v "$HOME/hockey:/hockey" --name dji_robomaster_ros dji_robomaster_ros:dev

## Run
Inside the container (the folder is mounted at /hockey):

    source /opt/ros/humble/setup.bash && source /opt/ros/ws/setup.bash
    ros2 daemon stop                      # avoid the ROS-daemon hang
    cd /hockey
    python3 goto_goal.py --robot 3 --robot-obj dji_robot_3 --goal hockey_sticks_1 \
        --v-max 0.12 --w-max 0.6          # start SLOW for the first run

The robot first rotates to aim, then drives straight, and stops automatically
within `--standoff` (default 0.40 m) of the target.
**Ctrl-C stops the robot** (publishes a zero Twist on exit).

## Before driving — verify the robot is live
Not every robot ID is actually connected (some only publish joint_states):

    ros2 topic list --no-daemon | grep -E '/robot3/(cmd_vel|state|connected)'
    ros2 topic echo --no-daemon /vrpn_mocap/dji_robot_3/pose   # values change when nudged

Use whichever robot ID is actually online; change --robot / --robot-obj to match.

## Troubleshooting
- **`ros2 topic list` hangs** → run `ros2 daemon stop` once per session, or add
  `--no-daemon` to ros2 commands (the daemon is slow/stuck under emulation).
- **It rotates the WRONG way (away from the target) during TURN** → the Vicon yaw
  axis doesn't match the robot's forward direction. Add a yaw correction and
  retry: `--yaw-offset 1.5708` (π/2), then `3.14159` (π), until it aims correctly.
- **It stalls mid-turn near the target** (small `w` can't overcome friction) →
  raise `--k-ang`, or ask for a `--w-min` floor to be added.
- **Robot doesn't move at all** → goal or robot pose not arriving. Check the two
  `ros2 topic echo` commands above; confirm object names with
  `ros2 topic list --no-daemon | grep vrpn_mocap`.
- Once direction is correct, raise `--v-max` toward the 0.35 default.

## Arguments
    --robot       robot id for /robotN/cmd_vel            (default 4)
    --robot-obj   vrpn_mocap object name for the robot    (default dji_robot_4)
    --goal        vrpn_mocap object to drive to           (default hockey_goal_1)
    --k-lin       linear gain (drive straight)            (default 0.8)
    --k-ang       angular gain (rotate in place)          (default 1.5)
    --v-max       max linear speed [m/s]                  (default 0.35)
    --w-max       max angular speed [rad/s]               (default 1.5)
    --standoff    stop this far from the target [m]       (default 0.40)
    --aim-tol     heading err to start driving [rad]      (default 0.05, ~3°)
    --hold-tol    heading err that forces re-aiming [rad] (default 0.20, ~11°)
    --yaw-offset  constant yaw correction [rad]           (default 0.0)
