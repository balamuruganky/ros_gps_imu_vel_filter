cmake_minimum_required(VERSION 2.8.3)
project(gps_imu_vel_filter)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
)

catkin_package()

add_subdirectory(ekf_nav_ins)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/include/
  ekf_nav_ins/inc
  ros_node/include
)

add_library(gps_imu_vel_filter
  ros_node/src/gps_imu_vel_filter.cpp
)

target_link_libraries(gps_imu_vel_filter
  ${catkin_LIBRARIES}
  ekf_nav_ins
)

## Localization node.
add_executable(${PROJECT_NAME}_node ros_node/src/filter_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node
  gps_imu_vel_filter
)
