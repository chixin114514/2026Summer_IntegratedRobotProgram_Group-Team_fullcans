#!/usr/bin/env bash
# Sync the source tree (which now contains the task3_six_fast package), copy the
# start script over, build the new package and run it.
# Nothing that already existed is modified: task3_six_fast is a new package, and
# the fast motion configuration lives inside it.
#
#   bash run_gui_fast.sh                    -> all six jobs, with GUI
#   HEADLESS=1 bash run_gui_fast.sh         -> all six jobs, no GUI
#   JOBS=P4 bash run_gui_fast.sh            -> only the listed jobs
#   JOBS=P1,P2 HEADLESS=1 bash run_gui_fast.sh
set -e
HOST=nvidia@192.168.31.146
LOCAL="/Users/a1523647308/overleaf modify/robot_fullcans/task3"
SRC="$LOCAL/ros2_ws/src/"
DEST="~/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws/src/"

rsync -az --delete-after --exclude '__pycache__' --exclude '*.pyc' \
  -e "ssh -o ConnectTimeout=10" "$SRC" "$HOST:$DEST"

scp -q -o ConnectTimeout=10 "$LOCAL/tools/task3_fast_start.sh" "$HOST:~/task3_fast_start.sh"

# The command line below deliberately contains no reference to the simulation
# package: the start script runs from a file, so its own `pkill -f task3_six`
# cannot match the shell that launched it.
ssh -o ConnectTimeout=10 "$HOST" \
  "HEADLESS=${HEADLESS:-0} JOBS='${JOBS:-}' bash \$HOME/task3_fast_start.sh"
