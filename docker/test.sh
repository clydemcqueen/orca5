#!/usr/bin/env bash

# test.sh - Run Orca5 automated end-to-end simulation tests inside Docker
#
# Usage:
#   ./test.sh [speedup] [graphics: auto|nvidia|intel|cpu]
#
# Examples:
#   ./test.sh               # Runs at speedup=1 with auto-detected graphics
#   ./test.sh 2             # Runs at speedup=2
#   ./test.sh 1 cpu         # Forces CPU software rendering

SPEEDUP="${1:-1}"
GRAPHICS="${2:-auto}"
TAG="orca5:sim"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source "$DIR/common.sh"

ensure_image "$TAG" sim "$DIR"

GRAPHICS=$(detect_graphics "$GRAPHICS")
echo "Using graphics mode: $GRAPHICS"

DOCKER_FLAGS="--rm \
    -v /etc/localtime:/etc/localtime:ro \
    -v $DIR/..:/home/orca5/colcon_ws/src/orca5 \
    --privileged"

# Apply graphics flags
DOCKER_FLAGS+=" $(get_graphics_flags "$GRAPHICS")"
DOCKER_FLAGS+=" $(get_dri_group_flags)"

echo "=================================================="
echo "Starting Orca5 automated tests in Docker ($TAG)..."
echo "Speedup factor: $SPEEDUP"
echo "=================================================="

docker run --entrypoint /bin/bash $DOCKER_FLAGS "$TAG" \
    /home/orca5/colcon_ws/src/orca5/scripts/run_test.bash "$SPEEDUP"
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
