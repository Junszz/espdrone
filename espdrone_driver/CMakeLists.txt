cmake_minimum_required(VERSION 2.8.3)

project(espdrone_driver)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  roslaunch
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_force_plugin scripts/force_plugin.cc)
target_link_libraries(gazebo_force_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
