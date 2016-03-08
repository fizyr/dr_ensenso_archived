cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	dr_log
	dr_util
	dr_camera
	dr_eigen
	dr_opencv
	dr_pcl
	pcl_ros
	pcl_conversions
)

find_package(Ensenso)
find_package(Boost)

catkin_package(
	INCLUDE_DIRS include ${Ensenso_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME} ${Ensenso_LIBRARIES}
	CATKIN_DEPENDS dr_camera
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${Ensenso_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/dump.cpp
	src/eigen.cpp
	src/ensenso.cpp
	src/error.cpp
	src/opencv.cpp
	src/pcl.cpp
	src/util.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Ensenso_LIBS})

add_executable(ensenso_viewer src/ensenso_viewer.cpp)
target_link_libraries(ensenso_viewer ${PROJECT_NAME} ${catkin_LIBRARIES} ${Ensenso_LIBRARIES})

add_executable(${PROJECT_NAME}_dump_tool src/dump_tool.cpp)
target_link_libraries(${PROJECT_NAME}_dump_tool ${PROJECT_NAME} ${catkin_LIBRARIES} ${Ensenso_LIBRARIES})

install(
	TARGETS "${PROJECT_NAME}" "${PROJECT_NAME}_dump_tool" "ensenso_viewer"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
