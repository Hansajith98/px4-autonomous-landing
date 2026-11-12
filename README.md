# Autonomous Drone Landing Using ROS 2 and PX4

An autonomous drone mission integrating **PX4, ROS 2, computer vision, and ArUco marker-based precision landing**. The system autonomously executes a waypoint navigation mission, detects a landing target using a downward-facing camera, aligns with the target, and performs a precision landing.

## Project Overview

This project demonstrates an autonomous UAV mission developed using ROS 2 and PX4. It combines waypoint navigation with vision-based precision landing in a simulated environment.

The drone first takes off, follows a predefined rectangular waypoint trajectory, and then uses an ArUco marker to identify and estimate the landing target's relative position. Based on the estimated target position, the precision landing controller guides the drone towards the marker before handing over the final landing sequence to PX4.

The complete mission is orchestrated by a single integrated ROS 2 mission controller.

## Mission Demonstration

### Autonomous Mission Sequence

1. **Autonomous Takeoff** – The drone takes off to an altitude of 2.5 metres.
2. **Waypoint Navigation** – Executes a predefined rectangular flight trajectory using PX4 Offboard control.
3. **Target Detection** – Detects an ArUco marker using the downward-facing camera.
4. **Target Pose Estimation** – Estimates the landing target's relative position.
5. **Precision Alignment** – Adjusts the drone's horizontal position to align with the landing target.
6. **Autonomous Landing** – Initiates the PX4 landing sequence and completes the mission.

### Mission Architecture

```text
                  ROS 2 Mission Controller
                            |
                 Autonomous Takeoff (2.5 m)
                            |
                    Waypoint Navigation
                            |
                     Landing Approach
                            |
                  Downward-Facing Camera
                            |
                     ArUco Detection
                            |
                   Target Pose Estimation
                            |
                  Precision Landing Control
                            |
                     PX4 Auto Land
                            |
                     Disarm / Mission End
```

## Key Features

- **Integrated Autonomous Mission:** Coordinates takeoff, waypoint navigation, target detection, precision alignment, and landing.
- **PX4 Offboard Control:** Controls the drone through PX4's ROS 2 interface.
- **Computer Vision:** Uses OpenCV and ArUco markers for visual landing-target detection.
- **Target Pose Estimation:** Estimates the relative position of the landing target using camera calibration and marker observations.
- **Precision Landing:** Corrects horizontal positioning based on visual target feedback.
- **ROS 2 Integration:** Uses ROS 2 nodes and topics for communication between mission, perception, and control components.
- **Software-in-the-Loop Simulation:** Developed and tested using PX4 SITL and Gazebo.

## Technology Stack

| Category | Technologies |
|---|---|
| Flight Controller | PX4 Autopilot |
| Robotics Middleware | ROS 2 Jazzy |
| Simulation | Gazebo Harmonic |
| Programming Language | Python |
| Computer Vision | OpenCV, ArUco |
| PX4 Communication | uXRCE-DDS, Micro XRCE-DDS Agent |
| ROS 2 Build System | colcon |
| Development Environment | Docker, WSL2, Ubuntu |
| Visualization | RViz, Gazebo |

## System Architecture

The system is organized into three main functional areas:

### 1. Mission Management and Flight Control

Responsible for managing the complete autonomous mission and communicating with PX4.

- Autonomous takeoff.
- Waypoint generation and navigation.
- PX4 Offboard control.
- Mission state management.
- Landing sequence initiation.

### 2. Computer Vision and Perception

Responsible for detecting and estimating the landing target using camera information.

- Live ArUco marker detection.
- Camera image processing.
- Camera calibration.
- Target pose estimation.
- Publishing target position information through ROS 2 topics.

### 3. Precision Landing

Uses the estimated target position to guide the drone towards the landing marker.

- Horizontal alignment.
- Landing approach.
- Position correction.
- Handover to PX4 Auto Land.

## Project Structure

```text
autonomous_landing_ws/
│
├── src/
│   ├── autonomous_mission/
│   │   ├── autonomous_mission/
│   │   │   ├── waypoint_follower.py
│   │   │   ├── live_aruco_detector.py
│   │   │   ├── aruco_pose_estimator.py
│   │   │   ├── precision_landing_controller.py
│   │   │   └── ...
│   │   │
│   │   ├── launch/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── px4_msgs/              # External PX4 dependency
│
├── build/
├── install/
├── log/
│
├── AUTONOMOUS_LANDING_QUICK_START.md
└── README.md
```

## Prerequisites

The development and simulation environment uses:

- Ubuntu 24.04 through WSL2.
- ROS 2 Jazzy.
- PX4 Autopilot release/1.18.
- Gazebo Harmonic.
- Docker and Docker Compose.
- Python 3.
- OpenCV 4.6.0.
- Micro XRCE-DDS Agent.

## Getting Started

### 1. Clone the repository

```bash
git clone <YOUR_REPOSITORY_URL>
cd autonomous_landing_ws
```

### 2. Retrieve the PX4 ROS 2 message dependency

The project uses the official PX4 message definitions from the matching release branch.

```bash
git clone --branch release/1.18 \
  https://github.com/PX4/px4_msgs.git src/px4_msgs
```

### 3. Build the ROS 2 workspace

```bash
source /opt/ros/jazzy/setup.bash

colcon build --packages-select autonomous_mission --symlink-install

source install/setup.bash
```

### 4. Run the autonomous mission

Follow the detailed instructions in:

**[Autonomous Landing Quick Start Guide](AUTONOMOUS_LANDING_QUICK_START.md)**

The guide covers the simulation startup sequence, Micro XRCE-DDS Agent, Gazebo camera bridge, camera transforms, perception nodes, and mission controller.

## Important ROS 2 Components

| Component | Responsibility |
|---|---|
| `waypoint_follower.py` | Integrated autonomous mission controller |
| `live_aruco_detector.py` | Real-time ArUco marker detection |
| `aruco_pose_estimator.py` | Landing target pose estimation |
| `precision_landing_controller.py` | Precision landing control component |
| `camera_transforms.launch.py` | Publishes camera-related TF transforms |

**Note:** The integrated mission is orchestrated by `waypoint_follower.py`. The individual takeoff and precision landing controllers are also available as separate components; do not run competing flight controllers simultaneously.

## Current Development Status

| Feature | Status |
|---|---|
| PX4 SITL integration | Completed |
| Gazebo camera integration | Completed |
| ROS 2 communication | Completed |
| ArUco marker detection | Completed |
| Landing target pose estimation | Completed |
| Autonomous takeoff | Completed |
| Waypoint navigation | Completed |
| Precision landing | Completed |
| Integrated autonomous mission | Tested successfully in simulation |

## Future Improvements

Potential directions for further development include:

- Testing with physical UAV hardware.
- Improving landing accuracy under different lighting conditions.
- Evaluating robustness against marker detection failures.
- Adding additional perception and landing-target detection methods.
- Integrating obstacle detection and avoidance.
- Evaluating landing performance under disturbances and varying initial conditions.

## Technical Notes

- The current implementation has been tested in simulation, not validated as a safety-certified real-world landing system.
- The landing target is an ArUco marker (ID: `0`).
- The marker size used in the simulation is 0.5 metres.
- PX4 uses the North-East-Down (NED) coordinate convention for local position control.
- The PX4 firmware and `px4_msgs` dependency should use matching release branches.

## Author

Developed as an autonomous drone systems engineering project, combining UAV control, robotics middleware, computer vision, and simulation.

**Technologies:** PX4 · ROS 2 · Python · OpenCV · Gazebo · Docker · Computer Vision

---

*This project is intended for research, learning, and experimentation in autonomous UAV systems.*