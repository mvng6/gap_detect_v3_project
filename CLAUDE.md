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

The `src/` directory is volume-mounted into the container at `/root/catkin_ws/src/`. Map files are stored inside `src/TR-200/woosh_navigation/maps/` and accessible in the container at `/root/catkin_ws/src/TR-200/woosh_navigation/maps/`.

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

## Architecture

### Package Structure

```
src/
├── main_command/          # Top-level coordination (Python)
├── doosan-robot/          # Doosan arm packages (C++ + Python)
│   ├── dsr_control/       # Hardware interface (C++)
│   ├── dsr_msgs/          # 100+ ROS service/message definitions
│   ├── dsr_description/   # URDF/Xacro models
│   ├── dsr_launcher/      # Launch files
│   ├── dsr_gazebo/        # Gazebo worlds
│   ├── dsr_example/       # Example scripts
│   └── moveit_config_*/   # MoveIt configs (one per robot model)
└── TR-200/                # Woosh mobile robot packages (Python)
    ├── woosh_robot_py/    # WebSocket SDK wrapper
    ├── woosh_bringup/     # ROS nodes + launch files
    ├── woosh_msgs/        # Custom service definitions
    ├── woosh_utils/       # Utility scripts (battery, etc.)
    ├── woosh_navigation/  # 네비게이션 패키지 모음
    │   └── AMCL/          # AMCL 로컬리제이션 (woosh_slam_amcl)
    │       └── maps/      # 맵 파일 (.pgm + .yaml)
    └── woosh_control/     # Placeholder
```

### Communication Architecture

```
main_command
├── /dsr01a0912/motion/move_joint  → dsr_control (C++ node → serial → Doosan arm)
└── /mobile_move                   → woosh_service_driver (Python → WebSocket → Woosh TR-200)
```

- **Doosan**: Serial over TCP port 12345 (real: 192.168.137.100, virtual: 127.0.0.1)
- **Woosh**: WebSocket at 169.254.128.2:5480, protobuf-serialized messages

### Key Source Files

| File | Role |
|------|------|
| `src/main_command/scripts/main_system_operation.py` | Top-level orchestration of both robots |
| `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py` | ROS service adapter for Woosh (implements `/mobile_move`) |
| `src/TR-200/woosh_bringup/scripts/woosh_rviz_debug.py` | RViz visualization node for Woosh |
| `src/TR-200/woosh_robot_py/woosh_robot.py` | Async WebSocket SDK for Woosh robot |
| `src/doosan-robot/dsr_control/src/dsr_hw_interface.cpp` | Doosan hardware abstraction layer |

### Custom ROS Services

**`/mobile_move`** (`woosh_msgs/MoveMobile`):
- Request: `float32 distance` (meters; negative = backward)
- Response: `bool success`, `string message`

**Doosan motion services** (namespace `dsr01a0912`):
- `motion/MoveJoint` — joint-space motion
- `motion/MoveTcp` — Cartesian motion
- 100+ additional services in `dsr_msgs/srv/`

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

## Maps

Map files are stored in `src/TR-200/woosh_navigation/maps/` and mounted into the container via the `src/` volume at `/root/catkin_ws/src/TR-200/woosh_navigation/maps/`. The Woosh robot uses these for AMCL-based localization.
