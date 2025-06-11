from setuptools import setup, find_packages

package_name = 'victor_sim_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='houhd',
    maintainer_email='houhd@umich.edu',
    description='Simulated hardware interface for Victor robot providing grouped command interfaces',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victor_robot_api_example = victor_sim_hardware.examples.simple_example:main',
        ],
    },
)
