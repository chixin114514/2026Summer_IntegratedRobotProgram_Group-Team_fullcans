"""启动 Task3 真机视觉分拣节点。

注意：本 launch 只拉起"分拣"这一侧。相机 + YOLO 检测节点是 Task1 的程序
（chixin114514/2026Summer 的 Task1/xby.py），需要另外单独启动，例如：

    cd ~/2026Summer/Task1 && python3 xby.py --model best.pt --camera 2

然后：

    ros2 launch task3_vision task3_vision.launch.py dry_run:=true   # 先只看解算
    ros2 launch task3_vision task3_vision.launch.py                 # 真跑
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "config", default_value="",
            description="vision_config.yaml 的绝对路径（留空用包内默认）"),
        DeclareLaunchArgument(
            "calibration_file", default_value="~/.ros/task3_table_calibration.yaml",
            description="桌面标定文件，由 calibrate_table 生成"),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = 只解算并打印，不让机械臂动"),
        DeclareLaunchArgument(
            "arm_speed_percent", default_value="0",
            description="关节速度百分比，0 = 用配置文件里的值"),
        DeclareLaunchArgument(
            "max_objects", default_value="0",
            description="最多处理几个物体，0 = 用配置文件里的值"),
        DeclareLaunchArgument(
            "passes", default_value="0",
            description="跑几轮，0 = 用配置文件里的值"),
    ]

    sort_node = Node(
        package="task3_vision",
        executable="vision_sort",
        name="task3_vision_sort",
        output="screen",
        parameters=[{
            "config": LaunchConfiguration("config"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "dry_run": LaunchConfiguration("dry_run"),
            "arm_speed_percent": LaunchConfiguration("arm_speed_percent"),
            "max_objects": LaunchConfiguration("max_objects"),
            "passes": LaunchConfiguration("passes"),
        }],
    )

    return LaunchDescription(arguments + [sort_node])
