cmake_minimum_required(VERSION 3.5)
project(vision_info_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(
  include
)

add_library(vision_info_parser 
  src/vision_info_parser.cpp)
ament_target_dependencies(vision_info_parser 
  rclcpp rclcpp_lifecycle lifecycle_msgs vision_msgs)

#add_executable(vision_info_server_node 
#  src/vision_info_server_node.cpp 
#  src/vision_info_server.cpp)
#ament_target_dependencies(vision_info_server_node 
#  rclcpp rclcpp_lifecycle lifecycle_msgs vision_msgs vision_info_parser)

# install executables/libs
install(TARGETS 
  #vision_info_server_node 
  vision_info_parser
  DESTINATION lib/vision_info_server
)

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()