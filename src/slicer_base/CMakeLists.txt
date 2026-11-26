cmake_minimum_required(VERSION 3.0.2)
project(slicer_base)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/driver_node.py scripts/gamepad_node.py scripts/velocity_mux.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

set(SLICER_C30D_HARDWARE_ROOT "${CMAKE_SOURCE_DIR}/../deps/slicer_ugv_hardware"
  CACHE PATH "Standalone Slicer hardware checkout included in the install")
if(EXISTS "${SLICER_C30D_HARDWARE_ROOT}/Host/rtk/c30d_link.py")
  find_package(Git REQUIRED)
  execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SLICER_C30D_HARDWARE_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE c30d_revision OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE c30d_result)
  if(NOT c30d_result EQUAL 0 OR NOT c30d_revision STREQUAL "842999ee65581d3c81756f52c012f2b6c4b9a609")
    message(FATAL_ERROR "C30D hardware checkout must match the revision pinned in dependencies.repos")
  endif()
  install(DIRECTORY "${SLICER_C30D_HARDWARE_ROOT}/Host/"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/c30d" FILES_MATCHING PATTERN "*.py")
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/base_smoke.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(FILES test/base.test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
  add_rostest(test/base.test)
endif()
