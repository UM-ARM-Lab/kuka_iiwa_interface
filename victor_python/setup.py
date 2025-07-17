from setuptools import find_packages, setup
from os.path import join, isfile
from glob import glob

package_name = 'victor_python'

# Only include existing launch files
launch_py_files = [f for f in glob(join('launch', '*.py')) if isfile(f)]
launch_xml_files = [f for f in glob(join('launch', '*.launch.xml')) if isfile(f)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share/', package_name), ['package.xml']),
        (join('share', package_name, 'launch'), launch_xml_files + launch_py_files),
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
            'arm_wrench_republisher.py = victor_python.arm_wrench_republisher:main',
            'mock_victor_ros.py = victor_python.mock_victor_ros:main',
            'launch_sim_vr = victor_python.launch_sim_vr:main',  # Alias for convenience
            'launch_right_real_vr = victor_python.launch_right_real_vr:main',  # Alias for convenience
        ],
    },
)
