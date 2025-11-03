# orca_bridge

The main event is the [slam_bridge.py](scripts/slam_bridge.py) node, which maintains MAVLink connection to ArduSub
and shuttles information between ROS2 and MAVLink.

## Theory of Operation (Life of a Pose)

This narrative follows the life of a single map -> base_link pose in simulation, showing how it
moves from system to system.

Gazebo calculates the ground truth pose of the sub using the Thruster, Buoyancy and Hydrodynamics plugins.

The ArduPilot Gazebo plugin interacts with the Gazebo model and sends the following to ArduSub SITL via UDP:
* pose
* angular velocity
* linear acceleration
* a simulated rangefinder value

Gazebo also generates a simulated camera image, which is published by the ros-gz bridge as `/image_raw`.

The ArduSub parameters control how the simulation data is interpreted, and in our case the following happens:
* pose.z is used to simulate a barometer
* pose.orientation is used to simulate two magnetometers
* angular velocity is used to simulate two gyroscopes
* linear acceleration is used to simulate two accelerometers
* the rangefinder value is used to simulate a sonar rangefinder
* (each simulated sensor has a noise model)

ArduSub publishes the sonar rangefinder reading in a `RANGEFINDER` MAVLink message.

orb_slam3_ros subscribes to `/image_raw` and calls ORB_SLAM3 to detect ORB features and compute a camera pose.

orb_slam3_ros publishes the status, the pose and a cloud of tracked points in `/slam_status`.
The tracked points are only those points that appear in the most recent camera image.
Since we are using Monocular mode the pose is only valid up to a scale factor.

The SlamBridge node subscribes to `/slam_status`, listens for `RANGEFINDER` messages and does a few things:
* compute a _visual_ rangefinder reading from the cloud of tracked points
* scale = sonar_rf / visual_rf
* delta_pose = previous_pose.inverse * current_pose
* send delta_pose to ArduSub in a `VISION_POSITION_DELTA` MAVLink message

ArduSub fuses the barometer, IMU and vision measurements into an estimated pose. 
This is sent as `LOCAL_POSITION_NED` (x, y, z) and `ATTITUDE` (roll, pitch, yaw) MAVLink messages.

SlamBridge listens for `LOCAL_POSITION_NED` and `ATTITUDE` messages and publishes various ROS2 messages and transforms.

ArduSub calculates servo commands to achieve the desired pose depending on the mode (ALT_HOLD, POSHOLD, AUTO).
These are sent to the ArduPilot Gazebo plugin via UDP.

ArduPilotPlugin sends servo commands to the Thruster plugins.

The ThrusterPlugins apply thrust forces in Gazebo.

## Coordinate Frames

All ROS world frames use the ENU convention: x is east, y is north, z is up.

ArduSub uses the NED convention: x is north, y is east, z is down.

ORB_SLAM3 uses the OpenCV convention: x is right, y is down, z is forward.

In the code all frames are ENU unless otherwise noted.

The variable name `t_target_source` refers to a 4x4 homogeneous matrix that will transform a vector from the `source` frame to the `target` frame.
This is consistent with the ORB_SLAM3 convention, but ORB_SLAM3 uses shorter names. E.g., `t_camera_world` is `Tcw`.

The `pose.Pose` class uses quaternions instead of 4x4 homogeneous matrices, and provides a `mult(other)` method instead of an operator.

### Transform Tree

I've tried to keep the frame names consistent with [ROS REP 105](https://www.ros.org/reps/rep-0105.html) and ORB_SLAM3,
and keep the names to a single word.
Here is the published ROS tf tree, showing the full frame name and the single word used in the code:

* map
  * base_link (base)
    * camera_link (link)
      * camera_sensor (camera)
    * ping_link (ping)
  * slam
    * world

### map

This is the world frame defined by ROS REP 105.

### base_link (base)

This is the frame of the robot as defined by ROS REP 105.

For the simulation this is defined by the [SDF file](orca_bringup/models/orca5/model.sdf.in) as the center of the thrust plane.

### camera_link (link)

This is defined by the [SDF file](orca_bringup/models/orca5/model.sdf.in) as the center of the camera sensor relative to base_link.
The camera is down-facing, so it is pitched forward by 90 degrees.

### camera_sensor (camera)

This is the camera frame used by ORB_SLAM3 and orb_slam3_ros.

### slam

This is the origin of the current SLAM map relative to map.

### world

This is the world frame used by ORB_SLAM3 and orb_slam3_ros.

### Debugging

You can see the ROS tf tree using the `tf2_monitor` tool. E.g., to view the tf tree in a simulation:
~~~
ros2 run tf2_ros tf2_monitor --ros-args -p use_sim_time:=True
~~~
