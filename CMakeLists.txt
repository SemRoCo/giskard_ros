cmake_minimum_required(VERSION 2.8.3)
project(giskard_examples)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  roslib 
  giskard 
  tf2_ros
  tf2_geometry_msgs
  interactive_markers 
  geometry_msgs
  giskard_msgs
  kdl_conversions)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib giskard tf2_ros tf2_geometry_msgs interactive_markers geometry_msgs giskard_msgs kdl_conversions
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

add_executable(pr2_controller src/${PROJECT_NAME}/pr2_controller.cpp)
target_link_libraries(pr2_controller ${catkin_LIBRARIES} yaml-cpp)

add_executable(pr2_controller_action_server src/${PROJECT_NAME}/pr2_controller_action_server.cpp)
target_link_libraries(pr2_controller_action_server ${catkin_LIBRARIES} yaml-cpp)

add_executable(controller_action_test_client src/${PROJECT_NAME}/controller_action_test_client.cpp)
target_link_libraries(controller_action_test_client ${catkin_LIBRARIES} yaml-cpp)

add_executable(single_pose_controller src/${PROJECT_NAME}/single_pose_controller.cpp)
target_link_libraries(single_pose_controller ${catkin_LIBRARIES} yaml-cpp)

add_executable(yaml_controller src/${PROJECT_NAME}/yaml_controller.cpp)
target_link_libraries(yaml_controller ${catkin_LIBRARIES} yaml-cpp)

add_executable(whole_body_interactive_markers src/${PROJECT_NAME}/interactive_marker.cpp)
target_link_libraries(whole_body_interactive_markers ${catkin_LIBRARIES})

add_executable(pose_stamped_transformer src/${PROJECT_NAME}/pose_stamped_transformer.cpp)
target_link_libraries(pose_stamped_transformer ${catkin_LIBRARIES})

add_executable(goal_transformer src/${PROJECT_NAME}/goal_transformer.cpp)
target_link_libraries(goal_transformer ${catkin_LIBRARIES})
