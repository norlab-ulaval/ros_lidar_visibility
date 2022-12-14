cmake_minimum_required(VERSION 2.8.3)
project(lidar_visibility)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  roscpp
  rospy
  tf2
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LambdaGrid.msg
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   nav_msgs 
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf2 message_runtime
)

###########
## Build ##
###########

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(visibility_node  src/main_mapper.cpp  src/LambdaMapper.cpp src/LambdaCell.cpp src/NormalEstimator.cpp src/Visibility.cpp)


add_dependencies(visibility_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(visibility_node
  ${catkin_LIBRARIES}
)

add_dependencies(visibility_node  ${catkin_EXPORTED_TARGETS})
