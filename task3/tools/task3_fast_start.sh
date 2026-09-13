#!/usr/bin/env bash
# Runs ON the target machine.  Kept as a file rather than an inline ssh command
# on purpose: an inline script puts its own text into the shell's command line,
# and then its own `pkill -f task3_six` matches and kills the shell running it.
#
#   HEADLESS=1 JOBS=P1,P2 bash task3_fast_start.sh
WS="$HOME/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws"
export ROS_DOMAIN_ID=107
export IGN_PARTITION=task3_six_validation_13
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
# Accept the grid filter as JOBS so the command line that starts this script never
# contains the package name either.
if [ -n "${JOBS:-}" ]; then export TASK3_SIX_JOBS="$JOBS"; fi

GUI_ARG="headless:=false"
if [ "${HEADLESS:-0}" = "1" ]; then GUI_ARG="headless:=true"; fi

source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --merge-install --packages-select task3_six_fast 2>&1 | tail -3
source install/setup.bash

pkill -f 'task3_si[x]' 2>/dev/null
sleep 3
rm -f "$HOME/.ros/task3_six_fast_sort_log.jsonl" /tmp/six_fast_launch.log
nohup ros2 launch task3_six_fast task3_six_fast_scene.launch.py "$GUI_ARG" \
  > /tmp/six_fast_launch.log 2>&1 &
sleep 25

echo "== procs =="
pgrep -af "ign gazebo" | head -4
echo "== nodes =="
ros2 node list
rm -f /tmp/sort_all_fast.log
nohup ros2 run task3_six_sim sort_all > /tmp/sort_all_fast.log 2>&1 &
echo "sort_all started (pid $!) -- log /tmp/sort_all_fast.log (jobs=${TASK3_SIX_JOBS:-all})"
