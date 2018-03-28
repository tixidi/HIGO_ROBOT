cmake_minimum_required(VERSION 2.8.3)
project(gohi_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  gohi_msgs
)




catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gohi_hw
#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/sdk/hf_link/
  include/sdk/robot_abstract/
)



 add_executable(gohi_hw_node src/main.cpp src/HIGO_ROS.cpp src/transport_serial.cpp src/HIGO_AP.cpp  include/sdk/hf_link/hf_link.cpp include/sdk/hf_link/hf_link_state_machine.cpp)
 add_dependencies(gohi_hw_node  gohi_msgs_gencpp)
 target_link_libraries(gohi_hw_node ${catkin_LIBRARIES})


