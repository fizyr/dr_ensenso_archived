cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	sensor_msgs
	geometry_msgs
)


add_message_files(
	DIRECTORY msg
)

add_service_files(
	DIRECTORY srv
)

generate_messages(
	DEPENDENCIES geometry_msgs sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs
	LIBRARIES ${PROJECT_NAME}
)

# Make a dummy library to force dependent targets to have dependencies on message generation.
configure_file("/dev/null" "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_library(${PROJECT_NAME} "${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
