## Launch files

### [sim.launch.py](launch/sim.launch.py)

Launch a Gazebo simulation.
Calls [bringup.launch.py](launch/bringup.launch.py).

To see parameters: `ros2 launch --show-args orca_bringup sim.launch.py`

### [hw.launch.py](launch/hw.launch.py)

Launch the nodes required to connect to the hardware.
Calls [bringup.launch.py](launch/bringup.launch.py).

To see parameters: `ros2 launch --show-args orca_bringup hw.launch.py`

### [bringup.launch.py](launch/bringup.launch.py)

Launch the core nodes, including orb_slam3_ros.