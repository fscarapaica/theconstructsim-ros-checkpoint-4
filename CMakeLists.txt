cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include/robot_gui
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_info_gui.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

add_executable(robot_gui_node src/robot_info_gui_node.cpp src/robot_info_gui.cpp)
target_link_libraries(robot_gui_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES})