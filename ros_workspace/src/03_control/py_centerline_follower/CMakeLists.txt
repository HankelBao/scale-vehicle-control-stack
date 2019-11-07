cmake_minimum_required(VERSION 2.8.3)
project(py_centerline_follower)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	common_msgs
	perception_msgs
	control_utilities
	common_utilities
	rviz_visual_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(

)

 install(PROGRAMS
	 nodes/py_centerline_follower
	 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	 )

