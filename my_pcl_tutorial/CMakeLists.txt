cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(voxel src/voxel.cpp)
add_executable(coefficients src/coefficients.cpp)
add_executable(passthrough src/passthrough.cpp)
add_executable(outliers src/outliers.cpp)
add_executable(detector src/detector.cpp)

target_link_libraries(voxel ${catkin_LIBRARIES})
target_link_libraries(coefficients ${catkin_LIBRARIES})
target_link_libraries(passthrough ${catkin_LIBRARIES})
target_link_libraries(outliers ${catkin_LIBRARIES})
target_link_libraries(detector ${catkin_LIBRARIES})