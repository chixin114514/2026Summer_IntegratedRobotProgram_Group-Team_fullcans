#!/usr/bin/env bash
# Restart the six-grid sim and run the six-object sort in the background.
#   bash run_all.sh                    -> all six jobs, fresh sim
#   TASK3_SIX_JOBS=P1 bash run_all.sh   -> only the listed jobs
#   SKIP_RESTART=1 bash run_all.sh      -> reuse the sim that is already up
WS=~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
export ROS_DOMAIN_ID=107
export IGN_PARTITION=task3_six_validation_13
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if [ "${SKIP_RESTART:-0}" != "1" ]; then
  pkill -f 'task3_si[x]' 2>/dev/null
  sleep 3
  rm -f "$HOME/.ros/task3_six_sort_log.jsonl" /tmp/six_launch.log
  nohup ros2 launch task3_six_sim task3_six_scene.launch.py headless:=true > /tmp/six_launch.log 2>&1 &
  sleep 15
fi

echo "== nodes =="
ros2 node list
rm -f /tmp/sort_all.log
nohup ros2 run task3_six_sim sort_all > /tmp/sort_all.log 2>&1 &
echo "sort_all started (pid $!) -- log /tmp/sort_all.log (jobs=${TASK3_SIX_JOBS:-all})"
