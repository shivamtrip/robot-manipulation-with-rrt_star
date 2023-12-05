# Install script for directory: /home/sridevi/Documents/GetABottle.AI/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sridevi/Documents/GetABottle.AI/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE PROGRAM FILES "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/_setup_util.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE PROGRAM FILES "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/env.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/setup.bash;/home/sridevi/Documents/GetABottle.AI/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE FILE FILES
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/setup.bash"
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/setup.sh;/home/sridevi/Documents/GetABottle.AI/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE FILE FILES
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/setup.sh"
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/setup.zsh;/home/sridevi/Documents/GetABottle.AI/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE FILE FILES
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/setup.zsh"
    "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/sridevi/Documents/GetABottle.AI/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/sridevi/Documents/GetABottle.AI/install" TYPE FILE FILES "/home/sridevi/Documents/GetABottle.AI/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/sridevi/Documents/GetABottle.AI/build/gtest/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/control_node/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/mimic_joint_gazebo_tutorial/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/multi_xarm5/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_bringup/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_description/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/dual_xarm6_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/lite6_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/uf_robot_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm5_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm5_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/xarm5_vacuum_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm6_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm6_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/xarm6_vacuum_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm7_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm7_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/xarm7_vacuum_gripper_moveit_config/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_msgs/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_sdk/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_vision/camera_demo/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/collision_check/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/planner_node/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/roboticsgroup_gazebo_plugins/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/roboticsgroup_upatras_gazebo_plugins/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/spawn_object/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/xarm7_redundancy_res/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_api/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/examples/run_recorded_traj/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_controller/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_gazebo/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_gripper/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_moveit_servo/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_planner/cmake_install.cmake")
  include("/home/sridevi/Documents/GetABottle.AI/build/xarm_ros/xarm_vision/d435i_xarm_setup/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sridevi/Documents/GetABottle.AI/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
