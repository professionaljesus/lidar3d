cmake_minimum_required(VERSION 3.0.2)
project(lidar3d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-g -Wall -O3)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  lfs_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
   LIBRARIES perception
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs lfs_msgs sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

set (LIDAR_RESOURCES
  src/utils.cpp
  src/visuals.cpp
  src/cluster.cpp
  src/ground_removal.cpp
  src/message_parsing.cpp
  src/classification.cpp
)
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN3_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(raw src/main.cpp ${LIDAR_RESOURCES})
add_dependencies(raw ${catkin_EXPORTED_TARGETS} lfs_msgs_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(raw ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
