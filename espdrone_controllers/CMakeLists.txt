cmake_minimum_required(VERSION 3.0.2)
project(espdrone_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    tf
    control_toolbox
    controller_interface
    geometry_msgs
    espdrone_interface
    espdrone_msgs
    roscpp
    tf2_geometry_msgs
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    LIBRARIES espdrone_controllers
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_library(espdrone_controllers
  src/attitude_controller.cpp
  src/velocity_controller.cpp
  src/position_controller.cpp
  )
target_link_libraries(espdrone_controllers ${catkin_LIBRARIES})
add_dependencies(espdrone_controllers ${catkin_EXPORTED_TARGETS})


install(TARGETS espdrone_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
