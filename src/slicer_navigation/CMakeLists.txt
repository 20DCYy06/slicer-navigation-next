cmake_minimum_required(VERSION 3.0.2)
project(slicer_navigation)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs angles base_local_planner costmap_2d dwa_local_planner
  geometry_msgs global_planner message_generation move_base_msgs nav_core nav_msgs pluginlib roscpp std_msgs tf2 tf2_geometry_msgs)
catkin_python_setup()
add_message_files(FILES PlannerVelocity.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

add_library(slicer_bspline_global_planner plugins/bspline_global_planner.cpp)
add_library(slicer_heading_dwa_planner plugins/dwa_heading_planner.cpp)
add_dependencies(slicer_heading_dwa_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
foreach(planner slicer_bspline_global_planner slicer_heading_dwa_planner)
  target_compile_features(${planner} PRIVATE cxx_std_14)
  target_include_directories(${planner} PRIVATE plugins/include ${catkin_INCLUDE_DIRS})
  target_link_libraries(${planner} ${catkin_LIBRARIES})
endforeach()
install(TARGETS slicer_bspline_global_planner slicer_heading_dwa_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugins/global_planner_plugin.xml plugins/local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/navigate_server.py scripts/run_move_base.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
