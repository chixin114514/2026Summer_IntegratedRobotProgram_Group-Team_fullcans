from glob import glob
from setuptools import find_packages, setup


package_name = "task3_six_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "sort_all = task3_six_sim.sort_all_client:main",
            "six_pick_sort_server = task3_six_sim.six_pick_sort_server:main",
        ],
    },
    zip_safe=True,
    maintainer="Integrated Robot Program Group Team",
    maintainer_email="team@example.com",
    description="Independent six-object yellow/green sorting simulation.",
    license="MIT",
)
