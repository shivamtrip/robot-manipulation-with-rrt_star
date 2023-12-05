# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xarm_gripper: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ixarm_gripper:/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xarm_gripper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" "xarm_gripper/MoveFeedback:actionlib_msgs/GoalID:xarm_gripper/MoveActionGoal:xarm_gripper/MoveActionResult:actionlib_msgs/GoalStatus:xarm_gripper/MoveResult:xarm_gripper/MoveGoal:xarm_gripper/MoveActionFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" "actionlib_msgs/GoalID:xarm_gripper/MoveGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header:xarm_gripper/MoveResult"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" "actionlib_msgs/GoalID:xarm_gripper/MoveFeedback:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_custom_target(_xarm_gripper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_gripper" "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_cpp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
)

### Generating Services

### Generating Module File
_generate_module_cpp(xarm_gripper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xarm_gripper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xarm_gripper_generate_messages xarm_gripper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_cpp _xarm_gripper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_gripper_gencpp)
add_dependencies(xarm_gripper_gencpp xarm_gripper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_gripper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)
_generate_msg_eus(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
)

### Generating Services

### Generating Module File
_generate_module_eus(xarm_gripper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xarm_gripper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xarm_gripper_generate_messages xarm_gripper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_eus _xarm_gripper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_gripper_geneus)
add_dependencies(xarm_gripper_geneus xarm_gripper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_gripper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)
_generate_msg_lisp(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
)

### Generating Services

### Generating Module File
_generate_module_lisp(xarm_gripper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xarm_gripper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xarm_gripper_generate_messages xarm_gripper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_lisp _xarm_gripper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_gripper_genlisp)
add_dependencies(xarm_gripper_genlisp xarm_gripper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_gripper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)
_generate_msg_nodejs(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
)

### Generating Services

### Generating Module File
_generate_module_nodejs(xarm_gripper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xarm_gripper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xarm_gripper_generate_messages xarm_gripper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_nodejs _xarm_gripper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_gripper_gennodejs)
add_dependencies(xarm_gripper_gennodejs xarm_gripper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_gripper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg"
  "${MSG_I_FLAGS}"
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)
_generate_msg_py(xarm_gripper
  "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
)

### Generating Services

### Generating Module File
_generate_module_py(xarm_gripper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xarm_gripper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xarm_gripper_generate_messages xarm_gripper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveAction.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveActionFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveGoal.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveResult.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/devel/share/xarm_gripper/msg/MoveFeedback.msg" NAME_WE)
add_dependencies(xarm_gripper_generate_messages_py _xarm_gripper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_gripper_genpy)
add_dependencies(xarm_gripper_genpy xarm_gripper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_gripper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_gripper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(xarm_gripper_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xarm_gripper_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_gripper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(xarm_gripper_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xarm_gripper_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_gripper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(xarm_gripper_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xarm_gripper_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_gripper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(xarm_gripper_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xarm_gripper_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_gripper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(xarm_gripper_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xarm_gripper_generate_messages_py std_msgs_generate_messages_py)
endif()
