from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'victor_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share/', package_name), ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*.launch.xml'))),
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
            'victor_vr_teleop.py = victor_python.victor_vr_teleop:main',
            'victor_command_gui.py = victor_python.victor_command_gui:main',
            'manual_motion.py = victor_python.manual_motion:main',
            'robotiq_grippers_joystick_node.py = victor_python.robotiq_grippers_joystick_node:main',
            'victor_joint_state_publisher.py = victor_python.joint_state_publisher:main',
            'arm_wrench_republisher.py = victor_python.arm_wrench_republisher:main',
            'mock_victor_ros.py = victor_python.mock_victor_ros:main',
        ],
    },
)
