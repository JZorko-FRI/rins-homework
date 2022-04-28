cmake_minimum_required(VERSION 3.0.2)
project(task2)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	OpenCV
	cv_bridge
	tf2_ros

	sensor_msgs
	visualization_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	laser_geometry
)

generate_messages(DEPENDENCIES std_msgs)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(task2_map_goals src/map_goals.cpp)
add_executable(task2_ring_detect src/ring_detect.cpp)
add_executable(task2_cylinder_detect src/cylinder_detect.cpp)

catkin_install_python(PROGRAMS
    scripts/face_localizer.py
	scripts/move_arm.py
	scripts/park_robot.py
	scripts/normal_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(task2_map_goals ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(task2_ring_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(task2_cylinder_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})

#include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_executable(map_goals src/map_goals.cpp)
#target_link_libraries(map_goals ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/detect_rings DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
