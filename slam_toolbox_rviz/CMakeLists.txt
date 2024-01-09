cmake_minimum_required(VERSION 3.5)
project(carto_toolbox_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  COMPONENTS
    rviz
    pluginlib
    carto_map
)
include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)


#include_directories(${EIGEN3_INCLUDE_DIRS})
#add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
      ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS
      rviz
)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(SOURCE_FILES 
  src/slam_toolbox_rviz_plugin.cpp 
)

qt_wrap_cpp(${PROJECT_NAME} MOC_FILES 
  src/slam_toolbox_rviz_plugin.h
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

