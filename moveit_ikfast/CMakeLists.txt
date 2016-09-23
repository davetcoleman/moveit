cmake_minimum_required(VERSION 2.8.3)
project(moveit_ikfast)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#catkin_python_setup()

install(
  PROGRAMS
    scripts/create_ikfast_moveit_plugin.py
    scripts/round_collada_numbers.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    templates
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)