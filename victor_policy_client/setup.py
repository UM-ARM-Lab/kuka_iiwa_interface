from setuptools import find_packages, setup
from os.path import join

package_name = 'victor_policy_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [join('resource', package_name)]),
        (join('share/', package_name), ['package.xml']),
        (join('share', package_name, 'examples'), ['examples/policy_client_examples.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'torch',
    ],
    zip_safe=True,
    maintainer='Daniel Hou',
    maintainer_email='houhd@umich.edu',
    description='Lightweight policy client for Victor robot with minimal dependencies',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victor_policy_client = victor_policy_client.victor_policy_client:main',
        ],
    },
)
