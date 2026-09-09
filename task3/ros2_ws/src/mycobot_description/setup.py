from glob import glob
from setuptools import find_packages, setup


package_name = "mycobot_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        (
            "share/" + package_name + "/urdf/mecharm_270_m5",
            glob("urdf/mecharm_270_m5/*.dae"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Integrated Robot Program Group Team",
    maintainer_email="team@example.com",
    description="Task3-local MechArm 270 mesh assets.",
    license="BSD-2-Clause",
)
