## Launch files

### [sim.launch.py](launch/sim.launch.py)

Launch a Gazebo simulation.
Calls [bringup.launch.py](launch/bringup.launch.py).

To see parameters: `ros2 launch --show-args orca_bringup sim.launch.py`

### [bringup.launch.py](launch/bringup.launch.py)

Bring up all core nodes, including orb_slam3_ros.

### Future: bench.launch.py

### Future: real.launch.py