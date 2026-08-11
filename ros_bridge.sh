#! /bin/bash

source deactivate py310

source /opt/carla_ws/ros_bridge_env.sh
source /opt/carla_ws/ros_bridge_ws/devel/setup.bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch