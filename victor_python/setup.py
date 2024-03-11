from setuptools import find_packages, setup

package_name = 'victor_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter',
    maintainer_email='pmitrano@umich.edu',
    description='python library and scripts for using victor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victor_vr_teleop = victor_python.victor_vr_teleop:main',
            'victor_command_guide = victor_python.victor_command_guide:main',
            'manual_motion = victor_python.manual_motion:main',
            'robotiq_grippers_joystick_node = victor_python.robotiq_grippers_joystick_node:main',
            'joint_state_publisher = victor_python.joint_state_publisher:main',
            'arm_wrench_republisher = victor_python.arm_wrench_republisher:main',
            'mock_victor_ros = victor_python.mock_victor_ros:main',
        ],
    },
)
