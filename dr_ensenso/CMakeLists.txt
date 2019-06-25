cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	pcl_ros
	pcl_conversions
	cmake_modules
)

find_package(Ensenso REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time chrono)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	message(STATUS "OPENMP FOUND")
	set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
	set(OpenMP_LIBS gomp)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(SYSTEM_LIBRARIES    ${Ensenso_LIBRARIES}    ${Boost_LIBRARIES}    ${Eigen_LIBRARIES}    ${OpenCV_LIBRARIES}    ${PCL_LIBRARIES})
SET(SYSTEM_INCLUDE_DIRS ${Ensenso_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${SYSTEM_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME} ${SYSTEM_LIBRARIES}
	CATKIN_DEPENDS
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${SYSTEM_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/eigen.cpp
	src/ensenso.cpp
	src/error.cpp
	src/opencv.cpp
	src/pcl.cpp
	src/util.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SYSTEM_LIBRARIES} ${OpenMP_LIBS})

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
