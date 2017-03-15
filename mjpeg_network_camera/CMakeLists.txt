cmake_minimum_required(VERSION 2.8.3)
project(mjpeg_network_camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  )

catkin_package(CATKIN_DEPENDS
  rospy
  sensor_msgs
  )
