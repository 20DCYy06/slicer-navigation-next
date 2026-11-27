cmake_minimum_required(VERSION 3.0.2)
project(slicer_operator_ui)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/operator_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
