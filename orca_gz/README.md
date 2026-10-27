# orca_gz

Gazebo simulation worlds, models, and launch files for Orca5.

## Launch files

### [gz.launch.py](launch/gz.launch.py)

Launch Gazebo, the ROS-Gazebo bridges, and ArduSub SITL (without ORB_SLAM3).
Called by [`orca_bringup/launch/sim.launch.py`](../orca_bringup/launch/sim.launch.py).

To see parameters: `ros2 launch --show-args orca_gz gz.launch.py`
