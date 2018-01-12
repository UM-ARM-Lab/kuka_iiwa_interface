#! /usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=['victor_fake_hardware_interface'],
    # scripts=['scripts/myscript'],
    package_dir={'': 'src'}
)

setup(**d)