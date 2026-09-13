from glob import glob
from setuptools import find_packages, setup


package_name = "task3_six_fast"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "six_pick_sort_fast_server = "
            "task3_six_fast.fast_pick_sort_server:main",
        ],
    },
    zip_safe=True,
    maintainer="Integrated Robot Program Group Team",
    maintainer_email="team@example.com",
    description="Faster variant of the six-object sorting simulation.",
    license="MIT",
)
