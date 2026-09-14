from setuptools import find_packages, setup

package_name = "fixed_joint_sorter"

setup(
    name=package_name,
    version="4.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/fixed_sorter.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="robot-team",
    maintainer_email="student@example.com",
    description="Fixed joint-angle sorter",
    license="MIT",
    entry_points={"console_scripts": ["fixed_joint_sort = fixed_joint_sorter.sorter_node:main"]},
)
