cmake_minimum_required(VERSION 3.5)
project(aipd_visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  aipd_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy aipd_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs sensor_msgs
)

catkin_install_python(PROGRAMS aipd_visualizer/lidar_bbox_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS aipd_visualizer/camera_bbox_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)