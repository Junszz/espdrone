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
  tf2 
)

include_directories(include ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES espdrone_controller
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs espdrone_msgs std_srvs tf2
    DEPENDS
)

###########
## Build ##
###########

add_library(espdrone_controller src/pid.cpp)
target_link_libraries(espdrone_controller ${catkin_LIBRARIES})
add_dependencies(espdrone_controller ${catkin_EXPORTED_TARGETS})

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller
   ${catkin_LIBRARIES}
)

add_executable(twist_controller src/twist_controller.cpp src/pid.cpp)
target_link_libraries(twist_controller
   ${catkin_LIBRARIES}
)
add_dependencies(twist_controller ${catkin_EXPORTED_TARGETS})
  

#############
## Install ##
#############

install(TARGETS twist_controller motor_controller espdrone_controller
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(dir config launch params meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)