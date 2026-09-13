from glob import glob

from setuptools import setup

package_name = "task3_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boff868",
    maintainer_email="boff868@users.noreply.github.com",
    description="Task3 real-robot vision sorting",
    license="MIT",
    entry_points={
        "console_scripts": [
            # 主分拣节点
            "vision_sort = task3_vision.vision_sort_node:main",
            # 桌面标定节点（像素 -> 桌面 XY）
            "calibrate_table = task3_vision.calibrate_table:main",
        ],
    },
)
