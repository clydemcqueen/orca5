#!/usr/bin/env bash

# run.sh - Run Orca5 Docker container for interactive use or commands
#
# Usage:
#   ./run.sh [target: sim|hw] [graphics: auto|nvidia|intel|cpu] [command...]
#
# Examples:
#   ./run.sh                  # Runs sim container with auto-detected graphics
#   ./run.sh sim nvidia       # Runs sim container with NVIDIA GPU acceleration
#   ./run.sh hw               # Runs hw container for topside computer

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source "$DIR/common.sh"

TARGET="${1:-sim}"
GRAPHICS="${2:-auto}"
TAG="orca5:${TARGET}"

ensure_image "$TAG" "$TARGET" "$DIR"

GRAPHICS=$(detect_graphics "$GRAPHICS")
echo "Using graphics mode: $GRAPHICS"

DOCKER_FLAGS="--rm \
    -v /etc/localtime:/etc/localtime:ro \
    -v $DIR/..:/home/orca5/colcon_ws/src/orca5 \
    --privileged"

# Apply graphics flags
DOCKER_FLAGS+=" $(get_graphics_flags "$GRAPHICS")"
DOCKER_FLAGS+=" $(get_dri_group_flags)"

# Joystick support
if [ -d "/dev/input" ]; then
    DOCKER_FLAGS+=" -v /dev/input:/dev/input"
fi

# X11 forwarding for GUI applications
if [ -n "$DISPLAY" ]; then
    XAUTH=/tmp/.docker.xauth
    if [ ! -f "$XAUTH" ]; then
        xauth_list=$(xauth nlist "$DISPLAY" 2>/dev/null)
        if [ -n "$xauth_list" ]; then
            xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
            echo "$xauth_list" | xauth -f "$XAUTH" nmerge - 2>/dev/null || touch "$XAUTH"
        else
            touch "$XAUTH"
        fi
        chmod a+r "$XAUTH"
    fi

    DOCKER_FLAGS+=" \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v $XAUTH:$XAUTH \
        -v /tmp/.X11-unix:/tmp/.X11-unix"
fi

# Select target container naming and network options
if [ "$TARGET" == "sim" ]; then
    echo "Configuring sim (simulation) container..."
    DOCKER_FLAGS+=" --name orca5_sim"
elif [ "$TARGET" == "hw" ]; then
    echo "Configuring hw (hardware) container..."
    DOCKER_FLAGS+=" --name orca5_hw --net=host"
fi

# Shift target and graphics parameters if provided
[ $# -gt 0 ] && shift
[ $# -gt 0 ] && shift

if [ $# -gt 0 ]; then
    echo "Running command in ${TAG}: $*"
    docker run $DOCKER_FLAGS "$TAG" "$@"
else
    echo "Starting interactive container ${TAG}..."
    docker run -it $DOCKER_FLAGS "$TAG"
fi
