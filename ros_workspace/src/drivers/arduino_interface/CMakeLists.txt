cmake_minimum_required(VERSION 2.8.3)
project(arduino_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_interface src/main.cpp src/serial_handler.cpp)
target_link_libraries(arduino_interface ${catkin_LIBRARIES})
add_dependencies(arduino_interface ${catkin_EXPORTED_TARGETS})

add_executable(arduino_interface_time_tester src/time_tester.cpp)
target_link_libraries(arduino_interface_time_tester ${catkin_LIBRARIES})
add_dependencies(arduino_interface_time_tester ${catkin_EXPORTED_TARGETS})

add_executable(arduino_interface_pwm_tester src/pwm_tester.cpp)
target_link_libraries(arduino_interface_pwm_tester ${catkin_LIBRARIES})
add_dependencies(arduino_interface_pwm_tester ${catkin_EXPORTED_TARGETS})
