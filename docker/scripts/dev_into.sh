#!/bin/bash

DEV_CONTAINER="carla_${USER}"

xhost +local:root 1>/dev/null 2>&1

docker exec -it  -u ${USER} ${DEV_CONTAINER} /bin/bash

xhost -local:root 1>/dev/null 2>&1
