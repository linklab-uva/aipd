cmake_minimum_required(VERSION 3.5)
project(speed_trap)

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

find_package(catkin REQUIRED)
find_package(rospy REQUIRED)
find_package(aipd_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy aipd_msgs geometry_msgs nav_msgs
)

catkin_install_python(PROGRAMS speed_trap/ego_velocity_estimator.py speed_trap/ticket_issuer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
