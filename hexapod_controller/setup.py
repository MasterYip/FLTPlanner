#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['hexapod_controller'],
    package_dir={'': 'scripts'},
    install_requires=['rospy', 'numpy', 'pyads'],
    scripts=[
        'scripts/hexapod201_interface.py',
        'scripts/hexapod_gait_demo.py',
        'scripts/test_hexapod_interface.py'
    ]
)

setup(**setup_args)
