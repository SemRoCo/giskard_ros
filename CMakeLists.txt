cmake_minimum_required(VERSION 2.8.3)
project(giskard_examples)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp roslib giskard)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib giskard
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
target_link_libraries(pr2_controller
  ${catkin_LIBRARIES} yaml-cpp)
