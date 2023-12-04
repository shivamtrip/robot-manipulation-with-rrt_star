# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xarm_msgs: 3 messages, 31 services")

set(MSG_I_FLAGS "-Ixarm_msgs:/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xarm_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" ""
)

get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_custom_target(_xarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xarm_msgs" "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_msg_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_msg_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)

### Generating Services
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_cpp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
)

### Generating Module File
_generate_module_cpp(xarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xarm_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xarm_msgs_generate_messages xarm_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_cpp _xarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_msgs_gencpp)
add_dependencies(xarm_msgs_gencpp xarm_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_msg_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_msg_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)

### Generating Services
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)
_generate_srv_eus(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
)

### Generating Module File
_generate_module_eus(xarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xarm_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xarm_msgs_generate_messages xarm_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_eus _xarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_msgs_geneus)
add_dependencies(xarm_msgs_geneus xarm_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_msg_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_msg_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)

### Generating Services
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)
_generate_srv_lisp(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
)

### Generating Module File
_generate_module_lisp(xarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xarm_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xarm_msgs_generate_messages xarm_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_lisp _xarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_msgs_genlisp)
add_dependencies(xarm_msgs_genlisp xarm_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_msg_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_msg_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)

### Generating Services
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)
_generate_srv_nodejs(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
)

### Generating Module File
_generate_module_nodejs(xarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xarm_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xarm_msgs_generate_messages xarm_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_nodejs _xarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_msgs_gennodejs)
add_dependencies(xarm_msgs_gennodejs xarm_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_msg_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_msg_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)

### Generating Services
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)
_generate_srv_py(xarm_msgs
  "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
)

### Generating Module File
_generate_module_py(xarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xarm_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xarm_msgs_generate_messages xarm_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/IOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/msg/CIOState.msg" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Move.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetAxis.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetInt16.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/TCPOffset.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ClearErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetErr.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperConfig.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperMove.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GripperState.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/ConfigToolModbus.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetControllerDigitalIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetControllerAnalogIO.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveAxisAngle.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelo.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/MoveVelocity.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetFloat32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetMultipleInts.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetString.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/PlayTraj.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetFloat32List.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetInt32.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/SetModbusTimeout.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/GetSetModbusData.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/Call.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtCaliLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sridevi/Documents/GetABottle.AI/src/xarm_ros/xarm_msgs/srv/FtIdenLoad.srv" NAME_WE)
add_dependencies(xarm_msgs_generate_messages_py _xarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xarm_msgs_genpy)
add_dependencies(xarm_msgs_genpy xarm_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xarm_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xarm_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xarm_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xarm_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xarm_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xarm_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xarm_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xarm_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xarm_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xarm_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xarm_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
