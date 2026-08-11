# Install script for directory: /opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/carla_ws/ros_bridge_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/carla_ackermann_control/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/carla_ackermann_control" TYPE FILE FILES "/opt/carla_ws/ros_bridge_ws/devel/include/carla_ackermann_control/EgoVehicleControlParameterConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/opt/carla_ws/ros_bridge_ws/devel/lib/python2.7/dist-packages/carla_ackermann_control/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/carla_ackermann_control" TYPE DIRECTORY FILES "/opt/carla_ws/ros_bridge_ws/devel/lib/python2.7/dist-packages/carla_ackermann_control/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/carla_ackermann_control/catkin_generated/installspace/carla_ackermann_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ackermann_control/cmake" TYPE FILE FILES
    "/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/carla_ackermann_control/catkin_generated/installspace/carla_ackermann_controlConfig.cmake"
    "/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/carla_ackermann_control/catkin_generated/installspace/carla_ackermann_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ackermann_control" TYPE FILE FILES "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/carla_ackermann_control" TYPE PROGRAM FILES
    "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control/src/carla_ackermann_control/carla_ackermann_control_node.py"
    "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control/src/carla_ackermann_control/carla_control_physics.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ackermann_control/launch" TYPE DIRECTORY FILES "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_ackermann_control/config" TYPE DIRECTORY FILES "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/carla_ackermann_control/config/")
endif()

