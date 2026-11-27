from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=["slicer_operator_ui"], package_dir={"": "src"}))
