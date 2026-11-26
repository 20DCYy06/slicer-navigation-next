cmake_minimum_required(VERSION 3.0.2)
project(slicer_perception)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/arm_cloud_filter.py scripts/self_scan_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
