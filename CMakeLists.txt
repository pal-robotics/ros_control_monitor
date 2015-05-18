cmake_minimum_required(VERSION 2.8.3)
project(ros_control_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_msgs
  controller_manager_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_control_motinor
#  CATKIN_DEPENDS controller_manager_msgs diagnostic_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(ros_control_monitor_node src/ros_control_monitor_node.cpp)

# add_dependencies(ros_control_motinor_node ros_control_motinor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_control_monitor_node  ${catkin_LIBRARIES})

#############
## Install ##
#############
 install(TARGETS ros_control_monitor_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
