#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

'''Do NOT manually run this file, insted use catkin'''
d = generate_distutils_setup(
    packages=['cpwalker_util'],
    package_dir={'': 'src'}
)
setup(**d)
