cmake_minimum_required(VERSION 3.0.2)
project(wit_node)

find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_devices
  ecl_sigslots
  ecl_time
  ecl_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pluginlib
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ImuGpsRaw.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  ecl_build
  ecl_devices
  ecl_geometry
  ecl_sigslots
  ecl_time
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
