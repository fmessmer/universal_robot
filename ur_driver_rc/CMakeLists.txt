cmake_minimum_required(VERSION 2.8.3)
project(ur_driver_rc)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  hardware_interface
  controller_manager
  control_msgs
)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ur_hardware_control_loop
    ur_controller_interface
    ur_hardware_interface
  CATKIN_DEPENDS 
    roscpp
    hardware_interface
    controller_manager
    ros_controllers
    control_msgs
  DEPENDS 
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Control loop
add_library(ur_hardware_control_loop src/ur_hardware_control_loop.cpp)
target_link_libraries(ur_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Controller Interface
add_library(ur_controller_interface src/ur_controller_interface.cpp)
target_link_libraries(ur_controller_interface
  ${catkin_LIBRARIES}
)

# Hardware Interface
add_library(ur_hardware_interface src/ur_hardware_interface.cpp)
target_link_libraries(ur_hardware_interface
  ur_controller_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(ur_driver_rc_node src/ur_driver_rc_node.cpp)
target_link_libraries(ur_driver_rc_node
  ur_hardware_interface
  ur_hardware_control_loop
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ur_driver_rc ur_driver_rc_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_driver_rc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
