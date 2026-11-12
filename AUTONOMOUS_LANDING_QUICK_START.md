# Autonomous Landing Mission --- Quick Start

This guide starts the ROS 2 autonomous waypoint and ArUco
precision-landing mission in the Docker environment.

## Environment

-   ROS 2 Jazzy
-   PX4 SITL
-   Gazebo Harmonic
-   Micro XRCE-DDS Agent
-   ROS 2 workspace: `/workspace/autonomous_landing_ws`
-   PX4 source: `/PX4-Autopilot`

> **Safety:** Run in SITL first. Keep the simulation, camera pipeline,
> and mission terminals running. Do not start another Offboard
> controller alongside `waypoint_follower`.

## Terminal 1 --- Start PX4 SITL and Gazebo

Inside the Docker container:

``` bash
cd /PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"
PX4_GZ_WORLD=lawn make px4_sitl gz_x500_mono_cam_down
```

Wait for PX4 and Gazebo to finish starting.

## Terminal 2 --- Start Micro XRCE-DDS Agent

``` bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

## Terminal 3 --- Start the Gazebo camera bridge

``` bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
'/world/lawn/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
'/world/lawn/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
```

## Terminal 4 --- Start camera TF transforms

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch autonomous_mission camera_transforms.launch.py
```

This launches the two static transforms:

-   `base_link` → `camera_link`
-   `camera_link` → `camera_optical_frame`

## Terminal 5 --- Start the live ArUco detector

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run autonomous_mission live_aruco_detector
```

## Terminal 6 --- Start the ArUco pose estimator

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run autonomous_mission aruco_pose_estimator
```

## Verify before starting the mission

Check PX4 local position:

``` bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

Confirm `xy_valid: true` and `z_valid: true`.

Check that the marker pose is being published when the landing marker is
visible:

``` bash
ros2 topic hz /landing/target_pose_body
```

The marker must be visible when the mission reaches its marker-search
phase.

## Terminal 7 --- Start the integrated mission

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run autonomous_mission waypoint_follower
```

The integrated mission controller is the single Offboard owner. Do
**not** run `takeoff_to_1m` or `precision_landing_controller` at the
same time.

## Mission sequence

1.  Pre-stream Offboard setpoints and arm.
2.  Take off to 2.5 m.
3.  Follow the local NED waypoint route:
    -   Starting position
    -   3 m North
    -   3 m North and 3 m East
    -   3 m East
    -   Return to the starting XY position
4.  Search for ArUco marker ID `0`.
5.  Align over the marker and descend.
6.  Request PX4 `AUTO_LAND`.
7.  PX4 completes landing and disarms.

NED convention: X = North, Y = East, Z = Down. Therefore, a negative Z
setpoint commands upward movement.

## Rebuild after changing package code or launch files

``` bash
cd /workspace/autonomous_landing_ws
colcon build --packages-select autonomous_mission --symlink-install
source install/setup.bash
```

Confirm the camera launch file was installed:

``` bash
ls install/autonomous_mission/share/autonomous_mission/launch/
```

Expected file:

``` text
camera_transforms.launch.py
```

## Troubleshooting checks

List active ROS 2 nodes:

``` bash
ros2 node list
```

Check PX4 telemetry rate:

``` bash
ros2 topic hz /fmu/out/vehicle_local_position_v1
```

Check the marker pose stream:

``` bash
ros2 topic hz /landing/target_pose_body
```

Check Offboard input topic publishers/subscribers:

``` bash
ros2 topic info /fmu/in/offboard_control_mode
ros2 topic info /fmu/in/trajectory_setpoint
ros2 topic info /fmu/in/vehicle_command
```

If the mission does not progress, inspect the mission-controller
terminal and PX4 status before attempting to arm or changing parameters.

## Important operating notes

-   Keep Terminals 1--6 running while the mission is active.
-   Start the mission only after PX4 telemetry and the camera/ArUco
    pipeline are available.
-   Ensure the landing marker is placed and visible before the vehicle
    reaches marker search.
-   Never run multiple nodes that publish competing PX4 Offboard
    setpoints.
-   This guide reflects the current SITL setup and topic/model names;
    update the Gazebo world, model, sensor topic names, and TF values if
    the simulation configuration changes.
