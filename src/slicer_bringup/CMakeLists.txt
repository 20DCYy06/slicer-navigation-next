cmake_minimum_required(VERSION 3.0.2)
project(slicer_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/mock_node.py scripts/status_node.py scripts/operator_ui.py scripts/record_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
