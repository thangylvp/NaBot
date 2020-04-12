cmake_minimum_required(VERSION 2.8.3)
project(peripheral_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
    tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES peripheral_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_angular src/test_angular.cpp)
target_link_libraries(test_angular ${catkin_LIBRARIES})

add_executable(test_cartographer src/test_cartographer.cpp)
target_link_libraries(test_cartographer ${catkin_LIBRARIES})

add_executable(test_controller src/test_controller.cpp src/header/controller.h src/header/controller.cpp)
target_link_libraries(test_controller ${catkin_LIBRARIES})

add_executable(fake_control src/fake_control.cpp)
target_link_libraries(fake_control ${catkin_LIBRARIES})
