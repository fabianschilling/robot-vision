cmake_minimum_required(VERSION 2.8.3)
project(vision_cv)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    image_transport
    cv_bridge
)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

# CPP stuff
include_directories(${catkin_INCLUDE_DIRS})
