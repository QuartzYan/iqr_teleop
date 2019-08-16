cmake_minimum_required(VERSION 2.8.3)
project(iqr_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iqr_teleop
#  CATKIN_DEPENDS joy roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_teleop
	src/joy_teleop.cpp)
target_link_libraries(joy_teleop 
  ${catkin_LIBRARIES})
  
add_executable(keyboard_teleop
	src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop 
	${catkin_LIBRARIES})