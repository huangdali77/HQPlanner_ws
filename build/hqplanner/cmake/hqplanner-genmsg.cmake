# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hqplanner: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ihqplanner:/home/zx414/HQPlanner_ws/src/hqplanner/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hqplanner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_custom_target(_hqplanner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hqplanner" "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hqplanner
  "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqplanner
)

### Generating Services

### Generating Module File
_generate_module_cpp(hqplanner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqplanner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hqplanner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hqplanner_generate_messages hqplanner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_dependencies(hqplanner_generate_messages_cpp _hqplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqplanner_gencpp)
add_dependencies(hqplanner_gencpp hqplanner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqplanner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hqplanner
  "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqplanner
)

### Generating Services

### Generating Module File
_generate_module_eus(hqplanner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqplanner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hqplanner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hqplanner_generate_messages hqplanner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_dependencies(hqplanner_generate_messages_eus _hqplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqplanner_geneus)
add_dependencies(hqplanner_geneus hqplanner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqplanner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hqplanner
  "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqplanner
)

### Generating Services

### Generating Module File
_generate_module_lisp(hqplanner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqplanner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hqplanner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hqplanner_generate_messages hqplanner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_dependencies(hqplanner_generate_messages_lisp _hqplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqplanner_genlisp)
add_dependencies(hqplanner_genlisp hqplanner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqplanner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hqplanner
  "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqplanner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hqplanner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqplanner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hqplanner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hqplanner_generate_messages hqplanner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_dependencies(hqplanner_generate_messages_nodejs _hqplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqplanner_gennodejs)
add_dependencies(hqplanner_gennodejs hqplanner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqplanner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hqplanner
  "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqplanner
)

### Generating Services

### Generating Module File
_generate_module_py(hqplanner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqplanner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hqplanner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hqplanner_generate_messages hqplanner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx414/HQPlanner_ws/src/hqplanner/msg/ref_line_test.msg" NAME_WE)
add_dependencies(hqplanner_generate_messages_py _hqplanner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hqplanner_genpy)
add_dependencies(hqplanner_genpy hqplanner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hqplanner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hqplanner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hqplanner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hqplanner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hqplanner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hqplanner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hqplanner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqplanner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hqplanner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hqplanner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqplanner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqplanner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hqplanner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hqplanner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
