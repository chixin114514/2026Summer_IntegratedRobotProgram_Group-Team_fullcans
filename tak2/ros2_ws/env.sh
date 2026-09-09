#!/bin/bash

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理其他 ROS workspace 遗留
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset PYTHONPATH

# ROS2 Humble
source /opt/ros/humble/setup.bash

# 当前项目
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

cd "$WS_DIR"

echo "Task2 ROS2 environment loaded"
echo "Workspace: $WS_DIR"
