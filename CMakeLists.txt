cmake_minimum_required(VERSION 2.8.3)
project(vision_info_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vision_info_server
  CATKIN_DEPENDS roscpp vision_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_info_server_node src/vision_info_server_node.cpp src/vision_info_server.cpp)
target_link_libraries(vision_info_server_node ${catkin_LIBRARIES})
add_dependencies(vision_info_server_node ${catkin_EXPORTED_TARGETS})

# install executables
install(TARGETS vision_info_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install launch files
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)