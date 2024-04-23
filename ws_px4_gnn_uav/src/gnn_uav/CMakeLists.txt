cmake_minimum_required(VERSION 3.5)
project(gnn_uav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gnn_uav_msgs REQUIRED)

add_executable(trajectory_setpoint_pub src/trajectory_setpoint_pub.cpp)
target_include_directories(trajectory_setpoint_pub PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_setpoint_pub rclcpp px4_msgs freyja_msgs gnn_uav_msgs)
install(TARGETS trajectory_setpoint_pub DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control_mode_pub src/offboard_control_mode_pub.cpp)
ament_target_dependencies(offboard_control_mode_pub rclcpp px4_msgs freyja_msgs gnn_uav_msgs)
install(TARGETS offboard_control_mode_pub DESTINATION lib/${PROJECT_NAME})

ament_package()
