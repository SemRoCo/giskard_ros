cmake_minimum_required(VERSION 2.8.3)
project(giskard_ros)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  roslib 
  giskard_core 
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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib giskard_core tf2_ros tf2_geometry_msgs interactive_markers geometry_msgs giskard_msgs kdl_conversions
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/${PROJECT_NAME}/whole_body_controller.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} yaml-cpp)

add_executable(whole_body_controller src/${PROJECT_NAME}/whole_body_controller_node.cpp)
target_link_libraries(whole_body_controller 
  ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME})
add_dependencies(whole_body_controller ${catkin_EXPORTED_TARGETS})

add_executable(controller_action_server src/${PROJECT_NAME}/controller_action_server.cpp)
target_link_libraries(controller_action_server ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(controller_action_server ${catkin_EXPORTED_TARGETS})

add_executable(controller_action_test_client src/${PROJECT_NAME}/controller_action_test_client.cpp)
target_link_libraries(controller_action_test_client ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(controller_action_test_client ${catkin_EXPORTED_TARGETS})

add_executable(float_array_separator src/${PROJECT_NAME}/float_array_separator.cpp)
target_link_libraries(float_array_separator ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(float_array_separator ${catkin_EXPORTED_TARGETS})

add_executable(single_pose_controller src/${PROJECT_NAME}/single_pose_controller.cpp)
target_link_libraries(single_pose_controller ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(single_pose_controller ${catkin_EXPORTED_TARGETS})

add_executable(yaml_controller src/${PROJECT_NAME}/yaml_controller.cpp)
target_link_libraries(yaml_controller ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(yaml_controller ${catkin_EXPORTED_TARGETS})

add_executable(whole_body_interactive_markers src/${PROJECT_NAME}/interactive_marker.cpp)
target_link_libraries(whole_body_interactive_markers ${catkin_LIBRARIES})
add_dependencies(whole_body_interactive_markers ${catkin_EXPORTED_TARGETS})

add_executable(pose_stamped_transformer src/${PROJECT_NAME}/pose_stamped_transformer.cpp)
target_link_libraries(pose_stamped_transformer ${catkin_LIBRARIES})
add_dependencies(pose_stamped_transformer ${catkin_EXPORTED_TARGETS})

add_executable(goal_transformer src/${PROJECT_NAME}/goal_transformer.cpp)
target_link_libraries(goal_transformer ${catkin_LIBRARIES})
add_dependencies(goal_transformer ${catkin_EXPORTED_TARGETS})

add_executable(goal_marker_viz src/${PROJECT_NAME}/goal_marker_viz.cpp)
target_link_libraries(goal_marker_viz ${catkin_LIBRARIES})
add_dependencies(goal_marker_viz ${catkin_EXPORTED_TARGETS})

add_executable(joint_state_pretty_printer src/${PROJECT_NAME}/joint_state_pretty_printer.cpp)
target_link_libraries(joint_state_pretty_printer ${catkin_LIBRARIES})
add_dependencies(joint_state_pretty_printer ${catkin_EXPORTED_TARGETS})

add_executable(yaml_executive src/${PROJECT_NAME}/yaml_executive.cpp)
target_link_libraries(yaml_executive ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(yaml_executive ${catkin_EXPORTED_TARGETS})

add_executable(check_controller_yaml src/${PROJECT_NAME}/check_controller_yaml.cpp)
target_link_libraries(check_controller_yaml ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(check_controller_yaml ${catkin_EXPORTED_TARGETS})
