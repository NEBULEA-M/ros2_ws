cmake_minimum_required(VERSION 3.8)
project(legion)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(motion src/motion.cpp)
ament_target_dependencies(motion rclcpp geometry_msgs turtlesim)

install(TARGETS
        motion
        DESTINATION lib/${PROJECT_NAME})

ament_package()
