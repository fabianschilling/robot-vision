cmake_minimum_required(VERSION 2.8.3)
project(ras_vision_pcl)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  roslaunch
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${catkin_LIBRARIES})

add_executable(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES})

add_executable(detector src/detector.cpp)
target_link_libraries(detector ${catkin_LIBRARIES})