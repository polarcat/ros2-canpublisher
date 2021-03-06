cmake_minimum_required(VERSION 3.5)

include(config.cmake OPTIONAL RESULT_VARIABLE config_var)

if(${config_var} STREQUAL "NOTFOUND")
	project(canbus-decoder)
else()
	# alternative project name should be set in config
	message("-- Using config.cmake")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} main.cpp)

ament_target_dependencies(${PROJECT_NAME}
	rclcpp std_msgs geometry_msgs sensor_msgs rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

# ros2 looks for package executables under /usr/lib/${PROJECT_NAME}/
install(TARGETS
	${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
