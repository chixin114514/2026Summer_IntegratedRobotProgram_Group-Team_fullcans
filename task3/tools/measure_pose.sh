#!/bin/bash
# Long-settle pose measurement: hold, let the soft PD converge, then read pads + joints.
# usage: measure_pose.sh "J1,...,J6" GRIP LABEL [hold_s] [read_at_s]
POSE="$1"; GRIP="$2"; LABEL="${3:-pose}"
HOLD="${4:-46}"; READ="${5:-40}"
export ROS_DOMAIN_ID=107
source /opt/ros/humble/setup.bash >/dev/null 2>&1
python3 /tmp/hold_pose.py --arm-prefix /task3_six/arm --grip-prefix /task3_six/gripper \
  --pose "$POSE" --grip "$GRIP" --seconds "$HOLD" >/tmp/hold_$LABEL.log 2>&1 &
HP=$!
sleep "$READ"
export IGN_PARTITION=task3_six_validation_13
echo "########## $LABEL pose=$POSE grip=$GRIP (t=${READ}s)"
echo "- joints (rad)"
timeout_cmd=""
ros2 topic echo --once --field position /task3_six/arm/joint_state 2>/dev/null | tr -d ' ' | tr '\n' ' '
echo
for l in gripper_left1 gripper_right1; do
  echo "- $l"
  ign model -m mecharm_270_six -l 2>/dev/null | grep -A14 "Name: $l\$" | grep -A2 "  - Pose"
done
wait $HP 2>/dev/null
exit 0
