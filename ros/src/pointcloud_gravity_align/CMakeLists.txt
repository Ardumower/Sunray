cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_gravity_align)

set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  #livox_ros_driver
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calib_B2W
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_gravity_align src/pointcloud_gravity_align_node.cpp)
target_link_libraries(pointcloud_gravity_align ${catkin_LIBRARIES})
