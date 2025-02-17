cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/localization_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
