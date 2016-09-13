cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso_node)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	dr_base
	dr_eigen
	dr_ensenso
	dr_ensenso_msgs
	dr_msgs
	dr_ros
	dr_thread
	image_transport
	pcl_conversions
	pcl_ros
	roscpp
)

catkin_package()

dr_include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(ensenso      src/ensenso.cpp src/timestamp.cpp)
add_executable(fake_ensenso src/fake_ensenso.cpp)
target_link_libraries(ensenso      ${catkin_LIBRARIES})
target_link_libraries(fake_ensenso ${catkin_LIBRARIES})

install(
	TARGETS ensenso fake_ensenso
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
