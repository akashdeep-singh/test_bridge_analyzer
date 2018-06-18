cmake_minimum_required(VERSION 2.8.3)
project(rr_cloud_bridge_analyzer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_message_files(
  FILES
  Packet.msg
  CloudBridgeTest.msg
)
add_service_files(
  FILES
    CloudBridgeTestService.srv
)
add_action_files(
  FILES
  CloudBridgeTestAction.action
)
generate_messages(
  DEPENDENCIES
  actionlib
  actionlib_msgs
  std_msgs
)

############
## catkin ##
############

catkin_package(
  LIBRARIES rr_cloud_bridge_analyzer
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/sender.py
  scripts/receiver.py
  scripts/cloud_bridge_test_server.py
  scripts/cloud_bridge_test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rr_cloud_bridge_analyzer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
