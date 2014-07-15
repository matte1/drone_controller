cmake_minimum_required(VERSION 2.8.3)
project(drone_controller)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    )

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
