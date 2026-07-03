cmake_minimum_required(VERSION 3.16.3)
project(socnav_msgs)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_COMPILER_ARG1 -std=c++17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedAgent.msg"
  "msg/TrackedAgents.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()