cmake_minimum_required(VERSION 2.8.3)
project(ras_vision_recognizer)

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
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Messages
add_message_files(FILES Rect.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

# CPP stuff
include_directories(${catkin_INCLUDE_DIRS})

add_executable(depth_detector src/detectors/depth_detector.cpp)
target_link_libraries(depth_detector ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(color_recognizer src/recognizers/color_recognizer.cpp)
target_link_libraries(color_recognizer ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# C++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
