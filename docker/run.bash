#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "`pwd`/../glider:/home/`whoami`/ws/src/glider" \
    -v "/home/jason/ROS/bags/:/home/`whoami`/data" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name glider-ros-noetic \
    glider-ros:dev \
    bash
xhost -
