cmake_minimum_required(VERSION 2.8.3)
project(rgbd_leg_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

add_executable(rgbd_leg_detector
  src/rgbd_leg_detector.cpp
  )

target_link_libraries(rgbd_leg_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
