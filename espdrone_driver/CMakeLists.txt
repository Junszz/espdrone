cmake_minimum_required(VERSION 2.8.3)
project(espdrone_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_force_plugin scripts/force_plugin.cc)
target_link_libraries(gazebo_force_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
