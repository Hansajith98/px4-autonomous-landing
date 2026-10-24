# Autonomous Flight Plan & Visual Landing --- Environment Startup

Quick-start commands for the existing WSL2 + Docker + ROS 2 Jazzy + PX4
SITL workspace.

## 1. Start Docker Compose (WSL terminal)

``` bash
cd ~/drone_ros2/workspaces/autonomous_landing_ws
docker compose -f docker/docker-compose.yml up -d
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
```

Expected container: `autonomous_landing_dev`, with UDP ports `14550`,
`18570`, and `14540` published.

## 2. Terminal 1 --- Start PX4 + Gazebo

Open a WSL terminal:

``` bash
docker exec -it autonomous_landing_dev bash
cd /PX4-Autopilot
make px4_sitl gz_x500
```

Wait for the PX4 shell and keep this terminal running. Do not run `make`
again if SITL is already running.

## 3. Terminal 2 --- Start the Micro XRCE-DDS agent

Open a second WSL terminal:

``` bash
docker exec -it autonomous_landing_dev bash
MicroXRCEAgent udp4 -p 8888
```

Keep this terminal running. PX4's DDS client should connect to the
agent.

## 4. Terminal 3 --- Prepare ROS 2 workspace

Open a third WSL terminal:

``` bash
docker exec -it autonomous_landing_dev bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Check that PX4 topics are available:

``` bash
ros2 topic list | grep '^/fmu/'
ros2 topic echo /fmu/out/vehicle_status_v4
```

Stop the topic echo with `Ctrl+C` after confirming messages arrive.

## 5. QGroundControl

Start QGroundControl on Windows and confirm that it connects to the
simulated X500. Keep it open to monitor the vehicle during SITL tests.

## 6. Build after changing code

Run inside the container:

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select autonomous_mission
source install/setup.bash
```

List the package executables:

``` bash
ros2 pkg executables autonomous_mission
```

## 7. Run the current waypoint controller (only when ready to test)

With PX4 SITL and the DDS agent still running, and QGroundControl
connected:

``` bash
cd /workspace/autonomous_landing_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run autonomous_mission waypoint_follower
```

The current prototype requests arming, Offboard mode, a short
rectangular route, and landing. Run it only in supervised simulation;
monitor QGroundControl throughout.

## 8. Shutdown

-   Stop a running ROS 2 node with `Ctrl+C`.
-   Stop the Micro XRCE-DDS agent with `Ctrl+C`.
-   Stop PX4/Gazebo with `Ctrl+C` in the PX4 terminal.
-   Stop the development container from WSL when finished:

``` bash
cd ~/drone_ros2/workspaces/autonomous_landing_ws
docker compose -f docker/docker-compose.yml down
```

`docker compose down` stops/removes the Compose container and network.
Workspace files persist because the workspace is bind-mounted.

## Environment notes

-   WSL workspace: `~/drone_ros2/workspaces/autonomous_landing_ws`
-   Container workspace: `/workspace/autonomous_landing_ws`
-   PX4 source in container: `/PX4-Autopilot`
-   ROS 2 distro: Jazzy
-   PX4 DDS agent: UDP port `8888`
-   PX4 SITL model: `gz_x500`
-   PX4 ROS 2 topics use `/fmu/in/...` and `/fmu/out/...`
-   PX4 output subscriptions in the Python node use Best Effort QoS.
-   This setup is for simulation. The user's physical FPV quad runs
    Betaflight and is not controlled by this PX4 Offboard project.
