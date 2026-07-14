#!/usr/bin/env bash
# Launch the RoboMaster ROS dev container (amd64 emulated) with WSLg GUI
# support and the project folders mounted.
#
# Usage:  bash ~/hockey/run_container.sh [image-tag]
#   default image tag: dji_robomaster_ros:dev  (the one you docker-committed)
set -e

IMAGE="${1:-dji_robomaster_ros:dev}"

# x86_64 emulation handler resets on every WSL restart -- re-register if missing.
if ! ls /proc/sys/fs/binfmt_misc/ 2>/dev/null | grep -q qemu-x86_64; then
  echo ">> Registering amd64 (x86_64) emulation..."
  sudo docker run --privileged --rm tonistiigi/binfmt --install amd64
fi

echo ">> Starting $IMAGE ..."
sudo docker run -it --rm --platform linux/amd64 \
  --network=host --pid=host --ipc=host \
  -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
  -e MPLCONFIGDIR=/hockey/.mplcache \
  -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v "$HOME/hockey:/hockey" \
  -v "$HOME/ros_ws:/ros_ws" \
  --name dji_robomaster_ros "$IMAGE"
