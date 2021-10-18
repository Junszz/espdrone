cmake_minimum_required(VERSION 3.0.2)
project(espdrone_interface)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs espdrone_msgs std_srvs controller_interface tf2 tf2_geometry_msgs urdf)
# remove hardware interface
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES espdrone_interface
    # remove hardware interface
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs espdrone_msgs std_srvs controller_interface
    DEPENDS
)

add_library(espdrone_interface src/quadrotor_interface.cpp src/helpers.cpp)
target_link_libraries(espdrone_interface ${catkin_LIBRARIES})
add_dependencies(espdrone_interface espdrone_msgs_generate_messages_cpp)

install(TARGETS espdrone_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
