cmake_minimum_required(VERSION 2.8.3)
project(homework4)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	cv_bridge
	tf2_ros

	sensor_msgs
	visualization_msgs
	nav_msgs
	std_msgs
	geometry_msgs
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(homework4_map_goals src/map_goals.cpp)
target_link_libraries(homework4_map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_executable(map_goals src/map_goals.cpp)
#target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/detect_rings DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
