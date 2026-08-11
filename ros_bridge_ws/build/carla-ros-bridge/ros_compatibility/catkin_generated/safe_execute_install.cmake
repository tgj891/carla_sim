execute_process(COMMAND "/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/ros_compatibility/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/ros_compatibility/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
