# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS1 Noetic robotics project integrating a **Doosan A0912 collaborative arm** with a **Woosh TR-200 mobile robot** for a gap detection system. Runs inside a Docker container on a NUC host.

- **Maintainer**: LDJ (djlee2@katech.re.kr) @ KATECH
- **ROS Version**: ROS1 Noetic (not ROS2)
- **Build System**: Catkin (CMake-based)
- All ROS commands run **inside the Docker container** at `/root/catkin_ws/`

## Docker Environment

```bash
# Host (one-time)
xhost +local:docker
docker-compose -f docker-compose.noetic_integration.yml up -d

# Enter container
docker exec -it noetic_robot_system_ws bash
source /root/catkin_ws/devel/setup.bash
```

`src/` is volume-mounted into the container at `/root/catkin_ws/src/`.

## Build Commands

```bash
# Full build
rosdep install --from-paths src --ignore-src -r -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build

# Single package
catkin build <package_name>

# Re-source after build
source devel/setup.bash
```

## Launch Commands

### Canonical mobile robot entry point

`woosh_navigation_system.launch` is the single official launch file. It passes all args as ROS params to `woosh_service_driver.py`, which orchestrates subprocess stacks at runtime.

```bash
# Base only (/mobile_move service)
roslaunch woosh_bringup woosh_navigation_system.launch

# AMCL + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# Cartographer fixed map + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=carto_fix navigation_mode:=move_base \
  state_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# GMapping SLAM + move_base (map-while-navigate)
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=gmapping navigation_mode:=move_base
```

Key args: `slam_mode` (`none`|`gmapping`|`cartographer`), `localization_mode` (`none`|`amcl`|`carto_fix`|`carto_nonfix`), `navigation_mode` (`none`|`costmap`|`move_base`), `map_file`, `state_file`, `launch_rviz` (default `true`), `global_planner_plugin`, `local_planner_plugin`, `nav_prerequisites_timeout` (default `30.0`).

**Constraint**: `slam_mode` and `localization_mode` are mutually exclusive — `_validate_modes()` enforces this.

### Doosan arm

```bash
# Virtual (no real robot)
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=virtual

# Real robot
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=real server_ip:=192.168.137.100

# Emulator (127.0.0.1:12345) — install once, then run
cd src/doosan-robot/doosan_robot && ./install_emulator.sh
cd src/doosan-robot/common/bin/DRCF && ./run_drcf.sh
```

### Integration

```bash
rosrun main_command main_system_operation.py   # both robots must be up
```

### SLAM / Map building

```bash
# GMapping
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2
roslaunch woosh_slam_gmapping save_map.launch map_name:=woosh_map

# Cartographer
roslaunch woosh_slam_cartographer cartographer.launch robot_ip:=169.254.128.2
roslaunch woosh_slam_cartographer save_map.launch map_name:=woosh_map

# Export map from robot's internal store
rosrun woosh_slam_amcl export_map.py _robot_ip:=169.254.128.2
```

Maps are saved to `src/TR-200/woosh_slam/maps/` (`.pgm` + `.yaml`; Cartographer also produces `.pbstream`).

## Architecture

### Communication

```
main_command
├── /dsr01a0912/motion/move_joint ──► dsr_control (C++) ──► Doosan A0912  [TCP 192.168.137.100:12345]
└── /mobile_move ──────────────────► woosh_service_driver (Python) ──► Woosh TR-200  [WS 169.254.128.2:5480]
```

### Sensor data flow

```
woosh_sensor_bridge.py  →  /scan, /odom, TF(odom→base_link)
                                ↓
SLAM/AMCL node  →  TF(map→odom), /map
```

`woosh_sensor_bridge.py` always runs (launched as a subprocess by `woosh_service_driver.py` in every mode). SLAM/AMCL launch files accept `launch_sensor_bridge:=false` to avoid duplicate start when called from `woosh_service_driver.py`.

### woosh_service_driver.py internal structure

The file is the runtime orchestrator for the entire mobile robot stack. Key classes:

| Class | Role |
|-------|------|
| `StackLauncher` | Manages subprocess lifecycle (sensor_bridge, SLAM, AMCL, costmap, move_base, RViz). Calls `SubprocessManager` internally. Has `wait_for_nav_prerequisites()` (waits for `/map` + TF `map→odom` before launching move_base) and `wait_for_costmap_ready()`. |
| `SmoothTwistController` | Implements `/mobile_move` service. Drives the robot via quintic minimum-jerk profile over WebSocket. Also handles `cmd_vel` passthrough for move_base integration (`enable_cmd_vel_passthrough()`). Odometry is twist-integrated (SDK provides no wheel encoders). |
| `SubprocessManager` | Launches/monitors named `roslaunch` subprocesses, restarts on crash. |
| `NavCsvLogger` | Logs `/cmd_vel` and SDK twist commands to CSV for post-analysis. |

`main()` flow: read ROS params (set by `woosh_navigation_system.launch`) → fall back to legacy CLI flags → translate to `slam_mode/localization_mode/navigation_mode` → `_validate_modes()` → start `StackLauncher` stacks in order → spin.

### Key source files

| File | Role |
|------|------|
| `src/main_command/scripts/main_system_operation.py` | Top-level sequence: Doosan A/B/C measurement points interleaved with `/mobile_move` calls |
| `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py` | Mobile robot orchestrator + `/mobile_move` service |
| `src/TR-200/woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` | WebSocket → `/scan`, `/odom`, TF |
| `src/TR-200/woosh_robot_py/woosh_robot.py` | Async WebSocket SDK |
| `src/TR-200/woosh_bringup/launch/woosh_navigation_system.launch` | Canonical launch entry point |
| `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/` | move_base, costmap, planner YAML configs |

### Custom ROS services

- **`/mobile_move`** (`woosh_msgs/MoveMobile`): `float32 distance` [m, negative=backward] → `bool success, string message`
- **`/dsr01a0912/motion/move_joint`** (`dsr_msgs/MoveJoint`): joint-space motion (absolute degrees, model a0912)
- **`/dsr01a0912/motion/move_tcp`** (`dsr_msgs/MoveTcp`): Cartesian motion

### TF tree

```
map → odom → base_link → laser
```
- `odom→base_link`: published by `woosh_sensor_bridge.py` (twist-integrated odometry, `transform_tolerance: 0.5s`)
- `base_link→laser`: static transform (z=0.25 m), published by `StackLauncher.start_base_laser_tf()`
- `map→odom`: published by SLAM or AMCL node

### Default file paths (inside container)

```
/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml         # AMCL map
/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream  # Cartographer state
```

## Python Dependencies (woosh_robot_py)

```
protobuf==4.21.0
websockets==12.0
typing-extensions>=4.0.0
python-dateutil>=2.8.2
```

## Supported Doosan Models

a0509, **a0912**, e0509, h2017, h2515, m0609, m0617, m1013, m1509 — project uses **a0912**.
