cmake_minimum_required(VERSION 3.0.2)
project(carto_map)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


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
find_package(PCL  REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
    LoadMap.srv
    SaveMap.srv
    OptimizeSubmap.srv
    OptimizeSubmapPose.srv
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
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${CMAKE_SOURCE_DIR}/lib)


add_executable(${PROJECT_NAME}
  src/main.cpp
  include/carto_map/scan_matching/real_time_correlative_scan_matcher_2d.cc
  include/carto_map/scan_matching/ceres_scan_matcher_2d.cc
  include/carto_map/scan_matching/fast_correlative_scan_matcher_2d.cc
)
  target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  ${CERES_LIBRARIES}
  cartographer
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  jsoncpp
)


# add_executable(test_node src/test.cpp)
# target_link_libraries(test_node
#   ${catkin_LIBRARIES}
#   ${boost_LIBRARIES}
#   jsoncpp
# )

