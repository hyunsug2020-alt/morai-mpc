#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["eskf"],
    # Keep implementation files directly in workspace src/eskf while
    # satisfying catkin's package-directory basename requirement.
    package_dir={"": ".."},
)

setup(**setup_args)
