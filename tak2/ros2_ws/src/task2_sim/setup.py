from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = 'task2_sim'


setup(
    name=package_name,

    version='0.4.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ],
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ],
        ),

        (
            'share/' + package_name + '/launch',
            glob(
                'launch/*.launch.py'
            ),
        ),

        (
            'share/' + package_name + '/worlds',
            glob(
                'worlds/*.sdf'
            ),
        ),

        (
            'share/' + package_name + '/config',
            glob(
                'config/*.yaml'
            ),
        ),

        (
            'share/' + package_name + '/urdf',
            glob(
                'urdf/*.urdf'
            ),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer=(
        'Integrated Robot Program Group Team'
    ),

    maintainer_email=(
        'team@example.com'
    ),

    description=(
        'Fixed-point pick-and-place simulation '
        'for MechArm 270.'
    ),

    license='MIT',

    entry_points={
        'console_scripts': [
            (
                'joint_controller = '
                'task2_sim.joint_controller:main'
            ),
        ],
    },
)
