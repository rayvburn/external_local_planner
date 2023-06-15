cmake_minimum_required(VERSION 3.12)
project(external_local_planner)

add_definitions(-std=c++17)

# prevent some unrelated warnings
include(FindPackageHandleStandardArgs)
set(FPHSA_NAME_MISMATCHED TRUE)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    cmake_modules
    costmap_2d
    nav_core
    nav_msgs
    pluginlib
    pcl_conversions
    roscpp
    tf2
    tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    base_local_planner
    nav_msgs
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/external_local_planner_ros.h
    src/external_local_planner_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## Install
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
