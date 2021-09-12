cmake_minimum_required(VERSION 2.8.3)
project(gx_sound_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
   FILES
   sound_signal.srv
)

add_action_files(
  DIRECTORY action
  FILES
  SoundRequest.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############
