# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS1 Noetic robotics project integrating a **Doosan collaborative arm robot** with a **Woosh TR-200 mobile robot** for a gap detection system. The project runs inside a Docker container (Ubuntu + ROS Noetic) on a NUC host.

- **Maintainer**: LDJ (djlee2@katech.re.kr) @ KATECH
- **ROS Version**: ROS1 Noetic (not ROS2)
- **Build System**: Catkin (CMake-based)

## Docker Environment

All development and execution happens inside the Docker container.

```bash
# Start container
docker-compose -f docker-compose.noetic_integration.yml up -d

# Enter container
docker exec -it noetic_robot_system_ws bash

# Inside container — source workspace
source /root/catkin_ws/devel/setup.bash
```

The `src/` directory is volume-mounted into the container at `/root/catkin_ws/src/`.

## Build Commands

Run inside the container at `/root/catkin_ws/`:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build

# Build a single package
catkin build <package_name>

# Source after build
source devel/setup.bash
```

## Launch Commands

```bash
# Woosh mobile robot — RViz visualization/debug
roslaunch woosh_bringup woosh_rviz_debug.launch robot_ip:=169.254.128.2

# Doosan arm — virtual mode (no real robot required)
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=virtual

# Doosan arm — combined Gazebo + RViz simulation
roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=a0912 mode:=virtual

# Doosan arm — real robot
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=real server_ip:=192.168.137.100

# Run main coordination node (requires both robots connected)
rosrun main_command main_system_operation.py
```

## Doosan Emulator (for development without real robot)

```bash
# Install emulator (one-time)
cd src/doosan-robot/doosan_robot && ./install_emulator.sh

# Run emulator (listens on 127.0.0.1:12345)
cd src/doosan-robot/common/bin/DRCF && ./run_drcf.sh
```

## SLAM (Map Building)

Three SLAM approaches are available for building maps with the Woosh TR-200:

```bash
# GMapping SLAM
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2

# Cartographer SLAM
roslaunch woosh_slam_cartographer cartographer.launch robot_ip:=169.254.128.2

# Save map (while SLAM is running, in a separate terminal)
roslaunch woosh_slam_gmapping save_map.launch map_name:=my_map
roslaunch woosh_slam_cartographer save_map.launch map_name:=my_map
```

## Localization (AMCL — requires an existing map)

```bash
roslaunch woosh_slam_amcl amcl.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_navigation/maps/woosh_map.yaml

# Generate map file from robot (if none exists yet)
rosrun woosh_slam_amcl export_map.py _robot_ip:=169.254.128.2
```

Map files (`.pgm` + `.yaml`) are stored in `src/TR-200/woosh_navigation/maps/` and accessible inside the container at `/root/catkin_ws/src/TR-200/woosh_navigation/maps/`.

## Architecture

### Package Structure

```
src/
├── main_command/                   # Top-level coordination (Python)
├── cartographer_src/               # Git submodules: cartographer + cartographer_ros
├── doosan-robot/                   # Doosan arm packages (C++ + Python)
│   ├── dsr_control/                # Hardware interface (C++)
│   ├── dsr_msgs/                   # 100+ ROS service/message definitions
│   ├── dsr_description/            # URDF/Xacro models
│   ├── dsr_launcher/               # Launch files
│   ├── dsr_gazebo/                 # Gazebo worlds
│   ├── dsr_example/                # Example scripts
│   └── moveit_config_*/            # MoveIt configs (one per robot model)
└── TR-200/                         # Woosh mobile robot packages (Python)
    ├── woosh_robot_py/             # WebSocket SDK wrapper
    ├── woosh_sensor_bridge/        # SDK sensor bridge (/scan, /odom, TF)
    ├── woosh_bringup/              # ROS nodes + launch files
    ├── woosh_msgs/                 # Custom service definitions
    ├── woosh_utils/                # Utility scripts (battery, etc.)
    ├── woosh_slam/                 # SLAM packages
    │   ├── GMapping/woosh_slam_gmapping/
    │   └── Cartographer/woosh_slam_cartographer/
    ├── woosh_navigation/
    │   ├── AMCL/                   # AMCL localization (woosh_slam_amcl)
    │   └── maps/                   # Map files (.pgm + .yaml)
    └── woosh_control/              # Placeholder
```

### Communication Architecture

```
main_command
├── /dsr01a0912/motion/move_joint  → dsr_control (C++ node → TCP → Doosan arm)
└── /mobile_move                   → woosh_service_driver (Python → WebSocket → Woosh TR-200)
```

- **Doosan**: TCP port 12345 (real: 192.168.137.100, virtual: 127.0.0.1)
- **Woosh**: WebSocket at 169.254.128.2:5480, protobuf-serialized messages

### Sensor Data Flow (SLAM / Localization)

```
woosh_sensor_bridge.py  →  /scan (LaserScan), /odom, TF(odom→base_link)
                                ↓
SLAM node (gmapping / cartographer / amcl)  →  TF(map→odom)
                                ↓
/map (OccupancyGrid)
```

`woosh_sensor_bridge.py` translates raw Woosh robot WebSocket data (LiDAR, odometry) into standard ROS topics. It is launched automatically by the SLAM/AMCL launch files.

### woosh_service_driver.py Modes

The driver is launched by the SLAM launch files and accepts an optional mode argument:

| Mode | Command | Usage |
|------|---------|-------|
| default | `rosrun woosh_bringup woosh_service_driver.py` | Normal `/mobile_move` service only |
| `slam` | `rosrun woosh_bringup woosh_service_driver.py slam` | Integrated with GMapping |
| `carto` | `rosrun woosh_bringup woosh_service_driver.py carto` | Integrated with Cartographer |

### Key Source Files

| File | Role |
|------|------|
| `src/main_command/scripts/main_system_operation.py` | Top-level orchestration: drives Doosan through A/B/C measurement points, interleaved with mobile robot moves |
| `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py` | ROS service adapter for Woosh (implements `/mobile_move` via `SmoothTwistController`) |
| `src/TR-200/woosh_bringup/scripts/woosh_rviz_debug.py` | RViz visualization node for Woosh |
| `src/TR-200/woosh_robot_py/woosh_robot.py` | Async WebSocket SDK for Woosh robot |
| `src/TR-200/woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` | Converts Woosh WebSocket sensor data to ROS `/scan`, `/odom`, and TF |
| `src/doosan-robot/dsr_control/src/dsr_hw_interface.cpp` | Doosan hardware abstraction layer |

### Custom ROS Services

**`/mobile_move`** (`woosh_msgs/MoveMobile`):
- Request: `float32 distance` (meters; negative = backward)
- Response: `bool success`, `string message`

**Doosan motion services** (namespace `dsr01a0912`):
- `motion/MoveJoint` — joint-space motion
- `motion/MoveTcp` — Cartesian motion
- 100+ additional services in `dsr_msgs/srv/`

### Gap Detection Sequence (main_system_operation.py)

The full measurement sequence (currently partially commented out — only A-points active):
1. Doosan → HOME position
2. Doosan traverses A-points (3 measurement poses)
3. Doosan → HOME
4. Mobile robot moves forward 0.3 m
5. *(commented)* Doosan traverses B-points (4 poses), mobile moves +0.6 m, C-points (5 poses), mobile returns −0.9 m

Joint positions are defined as absolute degree values for model **a0912**.

### Supported Doosan Robot Models

a0509, a0912, e0509, h2017, h2515, m0609, m0617, m1013, m1509

The project primarily uses **a0912**.

## Python Dependencies (woosh_robot_py)

```
protobuf==4.21.0
websockets==12.0
asyncio>=3.4.3
typing-extensions>=4.0.0
python-dateutil>=2.8.2
```
