cmake_minimum_required(VERSION 2.8.3)
project(i2c_imu)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  tf
  angles
)

find_library(RTIMULib libRTIMULib.so)

catkin_package(
  CATKIN_DEPENDS
      sensor_msgs
      roscpp
      tf
      angles
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(i2c_imu_node src/i2c_imu_node.cpp)

target_link_libraries(i2c_imu_node
  RTIMULib
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY
      launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
