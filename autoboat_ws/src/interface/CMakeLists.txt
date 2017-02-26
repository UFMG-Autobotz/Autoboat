cmake_minimum_required(VERSION 2.8.3)
project(interface)

#find_package(catkin REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  autoboat_msgs
)

catkin_package()

#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(nav_data src/fake_info.cpp) 
#target_link_libraries(nav_data ${catkin_LIBRARIES} )#${OpenCV_LIBS})

add_executable(teste_interface src/main.cpp lib/ros_functions.cpp lib/barco_class.cpp lib/arduino_class.cpp lib/utils.cpp lib/rs232_class.cpp lib/rs232lib.c) 
target_link_libraries(teste_interface ${catkin_LIBRARIES} )#${OpenCV_LIBS}) 
