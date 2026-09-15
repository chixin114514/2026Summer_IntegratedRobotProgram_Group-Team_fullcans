from glob import glob
from setuptools import find_packages, setup


package_name = "task3_six_fast"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "six_pick_sort_fast_server = "
            "task3_six_fast.fast_pick_sort_server:main",
            "vision_sort = task3_six_fast.vision_sort_client:main",
        ],
    },
    zip_safe=True,
    maintainer="Integrated Robot Program Group Team",
    maintainer_email="team@example.com",
    description="Fast six-object simulation with RGB colour-guided sorting.",
    license="MIT",
)
