cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	dr_log
	dr_util
	dr_camera
	dr_opencv
	pcl_ros
	pcl_conversions
)

find_package(OpenCV)
find_package(Ensenso)

catkin_package(
	INCLUDE_DIRS include ${Ensenso_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME} ${Ensenso_LIBRARIES}
	CATKIN_DEPENDS dr_camera
)

dr_include_directories(
	include
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${Ensenso_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/ensenso.cpp
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Ensenso_LIBS}
)

add_executable(ensenso_viewer src/ensenso_viewer.cpp)

target_link_libraries(ensenso_viewer
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Ensenso_LIBRARIES}
	${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
