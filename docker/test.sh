#!/usr/bin/env bash

# test.sh - Run Orca5 automated end-to-end simulation tests inside Docker
#
# EXPERIMENTAL!
#
# Usage:
#   ./test.sh [speedup] [graphics: nvidia|intel|cpu]
#
# Examples:
#   ./test.sh               # Runs at speedup=1 with auto-detected graphics
#   ./test.sh 2             # Runs at speedup=2
#   ./test.sh 1 cpu         # Forces CPU software rendering

SPEEDUP="${1:-1}"
GRAPHICS="${2:-auto}"
TAG="orca5:sim"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR" || exit 1

# Check if orca5:sim image exists
if ! docker image inspect "$TAG" >/dev/null 2>&1; then
    echo "Docker image '$TAG' not found. Building image..."
    ./build.sh sim || exit 1
fi

DOCKER_FLAGS="--rm \
    -v /etc/localtime:/etc/localtime:ro \
    -v $DIR/..:/home/orca5/colcon_ws/src/orca5 \
    --privileged"

# Auto-detect graphics if not specified
if [ "$GRAPHICS" == "auto" ]; then
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        GRAPHICS="nvidia"
    elif [ -d "/dev/dri" ]; then
        GRAPHICS="intel"
    else
        GRAPHICS="cpu"
    fi
    echo "Auto-detected graphics: $GRAPHICS"
fi

# Apply graphics flags
if [ "$GRAPHICS" == "nvidia" ]; then
    echo "Using NVIDIA GPU acceleration..."
    DOCKER_FLAGS+=" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --security-opt seccomp=unconfined \
        --gpus all"
elif [ "$GRAPHICS" == "intel" ] && [ -d "/dev/dri" ]; then
    echo "Using Intel GPU acceleration..."
    DOCKER_FLAGS+=" \
        -v /dev/dri:/dev/dri \
        --device /dev/dri"
else
    echo "Running with CPU software rendering..."
    DOCKER_FLAGS+=" \
        -e LIBGL_ALWAYS_SOFTWARE=1"
fi

if [ -d "/dev/dri" ]; then
    DOCKER_FLAGS+=" --group-add video"
    HOST_RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3)
    if [ -n "$HOST_RENDER_GID" ] && [ "$HOST_RENDER_GID" != "65534" ]; then
        DOCKER_FLAGS+=" --group-add $HOST_RENDER_GID"
    else
        DOCKER_FLAGS+=" --group-add 110"
    fi
    for gid in $(stat -c '%g' /dev/dri/* 2>/dev/null | sort -u); do
        if [ "$gid" != "65534" ] && [ "$gid" != "0" ]; then
            DOCKER_FLAGS+=" --group-add $gid"
        fi
    done
fi

echo "=================================================="
echo "Starting Orca5 automated tests in Docker ($TAG)..."
echo "Speedup factor: $SPEEDUP"
echo "=================================================="

# Test execution command inside the container
# TODO is there a better way to do this?
CONTAINER_CMD="
set -e
export PATH="/home/orca5/ardupilot/build/sitl/bin:$PATH"
source /opt/ros/jazzy/setup.bash
source /home/orca5/colcon_ws/install/setup.bash
if [ -f /home/orca5/colcon_ws/venv/bin/activate ]; then
    source /home/orca5/colcon_ws/venv/bin/activate
fi

cd /home/orca5/colcon_ws
echo '==> Building orca5 packages...'
colcon build --packages-select orca_msgs orca_bridge orca_bringup orca_test
source install/local_setup.bash

echo '==> Running launch tests (speedup=$SPEEDUP)...'
export ORCA_TEST_SPEEDUP=$SPEEDUP
colcon test --packages-select orca_test --event-handlers console_direct+

echo '==> Test results:'
colcon test-result --verbose
"

docker run --entrypoint /bin/bash $DOCKER_FLAGS "$TAG" -c "$CONTAINER_CMD"
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "=================================================="
    echo "All tests passed successfully!"
    echo "=================================================="
else
    echo "=================================================="
    echo "Tests failed with exit code $EXIT_CODE."
    echo "=================================================="
fi

exit $EXIT_CODE
