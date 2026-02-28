cmake_minimum_required(VERSION 3.16)
project(raptor_msgs)

# ROS2 Humble rosidl_generator_py 내부의 FindPythonInterp 경고 억제
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Grab all message files
file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR}
  msg/*.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_sources}
  srv/Transaction.srv
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
