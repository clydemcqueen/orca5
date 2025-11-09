#!/usr/bin/env bash

# Usage:
# ./run.sh <target> <graphics>

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Error: <target> and <graphics> arguments required."
  echo "Usage: ./run.sh <target> <graphics>"
  echo "where <target> can be sim or hw."
  echo "and <graphics> can be nvidia or intel."
  exit 1
fi

TARGET=$1
GRAPHICS=$2
TAG="orca5:${TARGET}"

# Set timezone
# Mount orca5 source
DOCKER_FLAGS="-it --rm \
    -v /etc/localtime:/etc/localtime:ro \
    -v ..:/home/orca5/colcon_ws/src/orca5 \
    --privileged"

# X11 forwarding
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
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

# Flags for GUI applications
DOCKER_FLAGS+=" \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v $XAUTH:$XAUTH \
  -v /tmp/.X11-unix:/tmp/.X11-unix"

# Select a container
if [ "$TARGET" == "sim" ]; then
  echo "Running sim (simulation) container..."
  DOCKER_FLAGS+=" --name orca5_sim"
elif [ "$TARGET" == "hw" ]; then
  echo "Running hw (hardware) container..."
  DOCKER_FLAGS+=" --name orca5_hw --net=host"
fi

# Select graphics
if [ "$GRAPHICS" == "nvidia" ]; then
  echo "Adding NVIDIA flags..."
  # For NVIDIA graphics. Install the NVIDIA Container Toolkit on the host:
  # https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
  DOCKER_FLAGS+=" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /dev/input:/dev/input \
    --security-opt seccomp=unconfined \
    --gpus all"
elif [ "$GRAPHICS" == "intel" ]; then
  echo "Adding Intel flags..."
  # For Intel graphics
  DOCKER_FLAGS+=" \
    -v /dev/dri:/dev/dri \
    --device /dev/dri"
else
  echo "Error: unsupported graphics option '$GRAPHICS'. Use 'nvidia' or 'intel'."
  exit 1
fi

echo "Starting container ${TAG}..."
docker run $DOCKER_FLAGS $TAG
