#!/bin/bash
DEV_CONTAINER="carla_${USER}"

docker commit ${DEV_CONTAINER} carlasim/carla:V1.2
