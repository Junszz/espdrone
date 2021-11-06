cmake_minimum_required(VERSION 3.0.2)
project(espdrone_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  espdrone_msgs 
  std_srvs 
  hardware_interface 
  controller_interface
)

include_directories(include ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES espdrone_controller
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs espdrone_msgs std_srvs hardware_interface controller_interface
    DEPENDS
)

###########
## Build ##
###########

add_library(espdrone_pose_controller src/espdrone_controller.cpp)
target_link_libraries(espdrone_pose_controller espdrone_controller)

#############
## Install ##
#############

install(TARGETS espdrone_controller espdrone_pose_controller espdrone_twist_controller espdrone_motor_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(dir config launch params meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)