cmake_minimum_required(VERSION 3.0.2)
project(simple_lo)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(simple_lo_node src/simple_lo_node.cpp)
target_link_libraries(simple_lo_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
