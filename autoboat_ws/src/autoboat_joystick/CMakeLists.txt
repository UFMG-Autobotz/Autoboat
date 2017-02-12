cmake_minimum_required(VERSION 2.8.3)
project(autoboat_joystick)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   Prop_msg.msg
   Stepper_msg.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoboat_joystick src/joystick_node.cpp)

target_link_libraries(autoboat_joystick
   ${catkin_LIBRARIES}
)

