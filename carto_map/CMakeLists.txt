cmake_minimum_required(VERSION 3.0.2)
project(carto_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  tf
  cartographer_ros_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(absl REQUIRED)
find_package(cartographer REQUIRED)
find_package(OpenCV REQUIRED )

add_service_files(
  DIRECTORY srv
  FILES
    LoadMap.srv
    SaveMap.srv
    OptimizeSubmap.srv
    RemoveSubmap.srv
    RemoveTrajectory.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carto_map
#  CATKIN_DEPENDS cartographer_ros_msgs geometry_msgs roscpp rospy std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${cartographer_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${CMAKE_SOURCE_DIR}/lib)


add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  ${CERES_LIBRARIES}
  cartographer
  ${OpenCV_LIBS}
  jsoncpp
)


# add_executable(test_node src/test.cpp)
# target_link_libraries(test_node
#   ${catkin_LIBRARIES}
#   ${boost_LIBRARIES}
#   jsoncpp
# )

