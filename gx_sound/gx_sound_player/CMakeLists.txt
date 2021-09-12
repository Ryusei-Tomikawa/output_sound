cmake_minimum_required(VERSION 2.8.3)
project(gx_sound_player)

find_package(catkin REQUIRED )

catkin_python_setup()

catkin_package(LIBRARIES gx_sound_player
  CATKIN_DEPENDS rospy std_msgs message_runtime                       
  DEPENDS system_lib)

## Generate services in the 'srv' folder
# add_service_files(
#    FILES
#    sound_signal.srv
# )

# generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
# )



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  node/sound_player
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch example
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_nosetests(test)
