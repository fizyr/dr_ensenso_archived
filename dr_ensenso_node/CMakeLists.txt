cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso_node)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	dr_ros
	dr_util
	dr_ensenso
	dr_msgs
	dr_eigen
	pcl_conversions
	pcl_ros
)

catkin_package()

dr_include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(ensenso src/ensenso.cpp)
target_link_libraries(ensenso ${catkin_LIBRARIES})

install(
	TARGETS "ensenso"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
