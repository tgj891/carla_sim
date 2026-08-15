#!/bin/bash

DEV_INSIDE="in-dev-docker"
DEV_CONTAINER="carla_${USER}"
USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
LOCAL_HOST=`hostname`
HOME_DIR=`pwd`

docker run -itd --rm --gpus all --name ${DEV_CONTAINER}       \
    --privileged --net=host                                         \
    --hostname "${DEV_INSIDE}"                                      \
    -v /etc/localtime:/etc/localtime:ro                             \
    -v /dev:/dev                                                    \
    -v /tmp/.X11-unix:/tmp/.X11-unix                                \
    -v /usr/lib/nvidia:/usr/lib/nvidia                              \
    -v /usr/lib/x86_64-linux-gnu/gtk-2.0:/usr/lib/x86_64-linux-gnu/gtk-2.0                   \
    -v /usr/lib/x86_64-linux-gnu/gtk-3.0:/usr/lib/x86_64-linux-gnu/gtk-3.0                   \
    -v /usr/lib/x86_64-linux-gnu/gstreamer-1.0:/usr/lib/x86_64-linux-gnu/gstreamer-1.0       \
    -v ${HOME_DIR}:/opt/carla_ws    \
    -e NVIDIA_VISIBLE_DEVICES=all                                   \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility    \
    -e DISPLAY=${DISPLAY}                                           \
    -e DOCKER_USER=${USER}                                          \
    -e USER=${USER}                                                 \
    -e DOCKER_USER_ID=${USER_ID}                                    \
    -e DOCKER_GRP=${GRP}                                            \
    -e DOCKER_GRP_ID=${GRP_ID}                                      \
    -e SDL_VIDEODRIVER=x11                             \
    -e GDK_SCALE                                                    \
    -e GDK_DPI_SCALE                                                \
    --add-host ${DEV_INSIDE}:127.0.0.1                              \
    --add-host ${LOCAL_HOST}:127.0.0.1                              \
    carlasim/carla:v1.0-beta /bin/bash

if [ "${USER}" != "root" ]; then
    docker exec ${DEV_CONTAINER} /bin/bash -c '/bin/bash /opt/carla_ws/docker/scripts/docker_adduser.sh'
fi
