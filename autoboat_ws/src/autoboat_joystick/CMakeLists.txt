cmake_minimum_required(VERSION 2.8.3)
project(autoboat_joystick)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
  autoboat_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoboat_joystick src/joystick_node.cpp)

target_link_libraries(autoboat_joystick
   ${catkin_LIBRARIES}
)

