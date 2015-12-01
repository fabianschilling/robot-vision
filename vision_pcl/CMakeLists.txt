cmake_minimum_required(VERSION 2.8.3)
project(vision_pcl)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  roslaunch
  image_transport
  cv_bridge
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${catkin_LIBRARIES})

add_executable(plane_segmentation src/plane_segmentation.cpp)
target_link_libraries(plane_segmentation ${catkin_LIBRARIES})

add_executable(euclidean_clustering src/euclidean_clustering.cpp)
target_link_libraries(euclidean_clustering ${catkin_LIBRARIES})

add_executable(transformation src/transformation.cpp)
target_link_libraries(transformation ${catkin_LIBRARIES})

add_executable(multiplane_segmentation src/multiplane_segmentation.cpp)
target_link_libraries(multiplane_segmentation ${catkin_LIBRARIES})

add_executable(ransac_model src/ransac_model.cpp)
target_link_libraries(ransac_model ${catkin_LIBRARIES})

add_executable(detector src/detector.cpp)
target_link_libraries(detector ${catkin_LIBRARIES})

add_executable(preprocess src/preprocess.cpp)
target_link_libraries(preprocess ${catkin_LIBRARIES})

add_executable(region_growing src/region_growing.cpp)
target_link_libraries(region_growing ${catkin_LIBRARIES})