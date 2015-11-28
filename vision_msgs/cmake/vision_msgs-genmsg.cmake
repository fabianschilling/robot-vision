# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vision_msgs: 8 messages, 0 services")

set(MSG_I_FLAGS "-Ivision_msgs:/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vision_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg" "vision_msgs/Box:vision_msgs/Histogram:vision_msgs/Centroid"
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg" ""
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg" ""
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg" ""
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg" ""
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg" ""
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg" "vision_msgs/Point"
)

get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg" NAME_WE)
add_custom_target(_vision_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision_msgs" "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg" "vision_msgs/Box:vision_msgs/Centroid"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)
_generate_msg_cpp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vision_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vision_msgs_generate_messages vision_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_cpp _vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_msgs_gencpp)
add_dependencies(vision_msgs_gencpp vision_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)
_generate_msg_lisp(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vision_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vision_msgs_generate_messages vision_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_lisp _vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_msgs_genlisp)
add_dependencies(vision_msgs_genlisp vision_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg;/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)
_generate_msg_py(vision_msgs
  "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(vision_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vision_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vision_msgs_generate_messages vision_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Detection.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Rect.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Box.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Point.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Histogram.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Centroid.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Points.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fabian/catkin_ws/src/ras_vision/vision_msgs/msg/Object.msg" NAME_WE)
add_dependencies(vision_msgs_generate_messages_py _vision_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_msgs_genpy)
add_dependencies(vision_msgs_genpy vision_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(vision_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(vision_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(vision_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(vision_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(vision_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(vision_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(vision_msgs_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(vision_msgs_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(vision_msgs_generate_messages_py geometry_msgs_generate_messages_py)
