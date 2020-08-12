cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

find_package(catkin REQUIRED message_generation std_msgs geometry_msgs trajectory_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})


add_definitions(-std=c++11)


add_message_files(
  FILES
  MotorRPM.msg
  OutputData.msg
  PositionCommand.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
#INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs trajectory_msgs 
)




install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
