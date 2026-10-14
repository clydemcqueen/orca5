#!/usr/bin/env bash

# run_test.bash - Run Orca5 automated tests inside the Docker container
#
# Usage:
#   run_test.bash [speedup]
#
# Examples:
#   ./src/orca5/scripts/run_test.bash       # Runs at speedup=1
#   ./src/orca5/scripts/run_test.bash 2     # Runs at speedup=2

set -e

SPEEDUP="${1:-1}"

# Ensure ArduPilot SITL binaries are on PATH
export PATH="/home/orca5/ardupilot/build/sitl/bin:$PATH"

# Source ROS 2 and workspace underlay
source /opt/ros/jazzy/setup.bash
source /home/orca5/colcon_ws/install/setup.bash

# Activate python virtual environment before build/test
if [ -f /home/orca5/colcon_ws/venv/bin/activate ]; then
    source /home/orca5/colcon_ws/venv/bin/activate
fi

cd /home/orca5/colcon_ws

echo "==> Building orca5 packages..."
colcon build --packages-select orca_msgs orca_bridge orca_bringup orca_test
source install/local_setup.bash

echo "==> Running launch tests (speedup=$SPEEDUP)..."
export ORCA_TEST_SPEEDUP="$SPEEDUP"
colcon test --packages-select orca_test --event-handlers console_direct+

echo "==> Test results:"
colcon test-result --verbose
