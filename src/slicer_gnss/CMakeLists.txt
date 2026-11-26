cmake_minimum_required(VERSION 3.0.2)
project(slicer_gnss)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/gnss_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch LICENSES DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
