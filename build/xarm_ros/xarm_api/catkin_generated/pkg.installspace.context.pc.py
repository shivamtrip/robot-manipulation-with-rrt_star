# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;actionlib;actionlib_msgs;control_msgs;xarm_sdk;xarm_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lxarm_ros_driver;-lxarm_ros_client".split(';') if "-lxarm_ros_driver;-lxarm_ros_client" != "" else []
PROJECT_NAME = "xarm_api"
PROJECT_SPACE_DIR = "/home/sridevi/Documents/GetABottle.AI/install"
PROJECT_VERSION = "0.0.0"
