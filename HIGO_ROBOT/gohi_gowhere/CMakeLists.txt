cmake_minimum_required(VERSION 2.8.3)
project(gohi_gowhere)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/NavSetPoint.cfg cfg/SimpleSetPoint.cfg)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gohi_gowhere
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


