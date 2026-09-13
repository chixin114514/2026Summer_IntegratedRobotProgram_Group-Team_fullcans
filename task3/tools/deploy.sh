#!/usr/bin/env bash
# Sync the ROS 2 source tree to the Jetson and rebuild.
set -e
HOST=nvidia@192.168.31.146
SRC="/Users/a1523647308/overleaf modify/robot_fullcans/task3/ros2_ws/src/"
DEST="~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws/src/"

run() {
  rsync -az --delete-after \
    --exclude '__pycache__' --exclude '*.pyc' \
    -e "ssh -o ConnectTimeout=10" "$SRC" "$HOST:$DEST"
  ssh -o ConnectTimeout=10 "$HOST" "bash -lc '
    set -e
    cd ~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --merge-install --packages-select task3_interfaces task3_sim task3_six_sim 2>&1 | tail -5
  '"
}
run
