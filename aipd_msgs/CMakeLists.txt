cmake_minimum_required(VERSION 3.5)
project(aipd_msgs)

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

# find dependencies
find_package(catkin REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(std_msgs REQUIRED)

add_message_files(DIRECTORY msg
		  FILES Ticket.msg  
		        DetectedObject.msg
		        DetectedObjectArray.msg
  
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package()
