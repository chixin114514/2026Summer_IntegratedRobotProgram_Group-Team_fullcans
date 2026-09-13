#!/usr/bin/env bash
# Launch the six-grid simulation WITH the Ignition GUI on the Jetson's monitor
# and then run the six-object sort, so the whole thing can be watched live.
#   bash run_gui.sh                    -> all six jobs
#   TASK3_SIX_JOBS=P1 bash run_gui.sh   -> only the listed jobs
WS=~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
export ROS_DOMAIN_ID=107
export IGN_PARTITION=task3_six_validation_13
# The desktop session is X11 on :1 (gdm owns the authority file).
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pkill -f 'task3_si[x]' 2>/dev/null
sleep 3
rm -f "$HOME/.ros/task3_six_sort_log.jsonl" /tmp/six_launch.log
nohup ros2 launch task3_six_sim task3_six_scene.launch.py headless:=false > /tmp/six_launch.log 2>&1 &
sleep 25

echo "== procs =="
pgrep -af 'ign gazebo' | head -5
echo "== nodes =="
ros2 node list
echo "== gui window =="
( wmctrl -l 2>/dev/null || xdotool search --name "Gazebo" 2>/dev/null ) | head -5
wmctrl -a "Gazebo" 2>/dev/null || true

rm -f /tmp/sort_all.log
nohup ros2 run task3_six_sim sort_all > /tmp/sort_all.log 2>&1 &
echo "sort_all started (pid $!) -- log /tmp/sort_all.log (jobs=${TASK3_SIX_JOBS:-all})"
