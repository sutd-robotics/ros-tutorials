cmake_minimum_required(VERSION 2.8.3)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  msg_example_msgs
)

catkin_package()
catkin_python_setup()
