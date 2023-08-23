cmake_minimum_required(VERSION 3.0.2)
project(path_tracking_py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS posez)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  actionlib
  move_base_msgs
  tf
  # grid_map_ros
)

find_package(jsoncpp REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_tracking
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_tracking   
  src/main.cpp
  src/tracking.cpp
  src/pure_pursuit.cpp
  src/reference_plan.cpp
  src/ros_interface.cpp
  src/robot.cpp
  src/utils.cpp
)

add_executable(map_grid_test   
  src/map.cpp
)

add_executable(tracking_sim   
  src/tracking_sim.cpp
  src/reference_plan.cpp
  src/ros_interface.cpp
  src/robot.cpp
  src/utils.cpp
)

add_dependencies(path_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracking ${catkin_LIBRARIES} jsoncpp)

add_dependencies(map_grid_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_grid_test ${catkin_LIBRARIES})

add_dependencies(tracking_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_sim ${catkin_LIBRARIES} jsoncpp)
