# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xarm_planner: 0 messages, 4 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xarm_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_custom_target(_xarm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_planner" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_custom_target(_xarm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_planner" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_custom_target(_xarm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_planner" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_custom_target(_xarm_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_planner" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
)
_generate_srv_cpp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
)
_generate_srv_cpp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
)
_generate_srv_cpp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
)

### Generating Module File
_generate_module_cpp(xarm_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xarm_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xarm_planner_generate_messages xarm_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_cpp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_cpp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_cpp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_cpp _xarm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_planner_gencpp)
add_dependencies(xarm_planner_gencpp xarm_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
)
_generate_srv_eus(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
)
_generate_srv_eus(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
)
_generate_srv_eus(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
)

### Generating Module File
_generate_module_eus(xarm_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xarm_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xarm_planner_generate_messages xarm_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_eus _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_eus _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_eus _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_eus _xarm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_planner_geneus)
add_dependencies(xarm_planner_geneus xarm_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
)
_generate_srv_lisp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
)
_generate_srv_lisp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
)
_generate_srv_lisp(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
)

### Generating Module File
_generate_module_lisp(xarm_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xarm_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xarm_planner_generate_messages xarm_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_lisp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_lisp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_lisp _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_lisp _xarm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_planner_genlisp)
add_dependencies(xarm_planner_genlisp xarm_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
)
_generate_srv_nodejs(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
)
_generate_srv_nodejs(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
)
_generate_srv_nodejs(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
)

### Generating Module File
_generate_module_nodejs(xarm_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xarm_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xarm_planner_generate_messages xarm_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_nodejs _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_nodejs _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_nodejs _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_nodejs _xarm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_planner_gennodejs)
add_dependencies(xarm_planner_gennodejs xarm_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
)
_generate_srv_py(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
)
_generate_srv_py(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
)
_generate_srv_py(xarm_planner
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
)

### Generating Module File
_generate_module_py(xarm_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xarm_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xarm_planner_generate_messages xarm_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/pose_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_py _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/joint_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_py _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/exec_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_py _xarm_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_planner/srv/single_straight_plan.srv" NAME_WE)
add_dependencies(xarm_planner_generate_messages_py _xarm_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_planner_genpy)
add_dependencies(xarm_planner_genpy xarm_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(xarm_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xarm_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(xarm_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xarm_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(xarm_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xarm_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(xarm_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xarm_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(xarm_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xarm_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
