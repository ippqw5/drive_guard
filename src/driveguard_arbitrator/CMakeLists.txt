cmake_minimum_required(VERSION 3.8)
project(driveguard_arbitrator)

include_directories(${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(driveguard_arbitrator_node
        src/boost/sml.hpp
        src/rclcpp/driveguard_arbitrator_node.cpp
        src/rclcpp/driveguard_arbitrator_node.h
        src/config/driveguard_arbitrator_config.cpp
        src/config/driveguard_arbitrator_config.h
)

ament_target_dependencies(driveguard_arbitrator_node
        rclcpp
        geometry_msgs
        std_msgs
        lifecycle_msgs
        action_msgs
        nav2_msgs
        tf2_ros
)

install(TARGETS
  driveguard_arbitrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
