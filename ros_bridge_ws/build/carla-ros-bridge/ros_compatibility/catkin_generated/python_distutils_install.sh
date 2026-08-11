#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/ros_compatibility"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/opt/carla_ws/ros_bridge_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/opt/carla_ws/ros_bridge_ws/install/lib/python2.7/dist-packages:/opt/carla_ws/ros_bridge_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/opt/carla_ws/ros_bridge_ws/build" \
    "/usr/bin/python2" \
    "/opt/carla_ws/ros_bridge_ws/src/carla-ros-bridge/ros_compatibility/setup.py" \
     \
    build --build-base "/opt/carla_ws/ros_bridge_ws/build/carla-ros-bridge/ros_compatibility" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/opt/carla_ws/ros_bridge_ws/install" --install-scripts="/opt/carla_ws/ros_bridge_ws/install/bin"
