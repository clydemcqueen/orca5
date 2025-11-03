#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Specific for NVIDIA drivers. Install the NVIDIA Container Toolkit for Ubuntu 24.04:
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
# Mount orca5 source
docker run -it \
    --rm \
    --name orca5 \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "..:/home/orca5/colcon_ws/src/orca5" \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    orca5:latest