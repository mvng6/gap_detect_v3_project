# ROS1 Navigation Refactoring Prompt

You are a senior ROS engineer responsible for refactoring a ROS1 Noetic-based mobile robot navigation / SLAM / localization stack.
From this point onward, assume that you are directly modifying my GitHub repository `gap_detect_v3_project`, and produce **patch-ready, immediately applicable code** rather than a high-level design memo.

## Important

- Do **not** migrate this project to ROS2/Nav2. Keep the system on **ROS1 Noetic + roslaunch + move_base**.
- My goal is to replace the current CLI-style execution pattern such as:
  - `rosrun woosh_bringup woosh_service_driver.py ...`
  with a **single canonical launch entry point**:
  - `roslaunch woosh_bringup woosh_navigation_system.launch ...`
- Do not provide only conceptual guidance. Reflect the refactor in actual code-level changes across launch files, YAML, CMake, package metadata, and README where needed.
- Preserve the existing behavior as much as possible, but refactor the project into a **launch-first, parameter-driven orchestration structure**.
- However, runtime sequencing such as topic/TF readiness waiting, single SDK ownership, and `/cmd_vel` passthrough is already handled reasonably well inside `woosh_service_driver.py`.
- Therefore, prefer a **hybrid architecture** in which:
  - `roslaunch` is the configuration surface.
  - `woosh_service_driver.py` remains the runtime orchestrator.
- In other words, do **not** try to encode all sequencing using complex `$(eval ...)` launch logic only. Instead, refactor `woosh_service_driver.py` from a **CLI-centered orchestrator** into a **launch-parameter-centered orchestrator**.

---

## Repository Context

Current key packages/files:

- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
- `src/TR-200/woosh_bringup/launch/`
- `src/TR-200/woosh_navigation/AMCL/launch/amcl.launch`
- `src/TR-200/woosh_navigation/AMCL/config/amcl_params.yaml`
- `src/TR-200/woosh_navigation/Costmap/woosh_costmap/launch/global_costmap.launch`
- `src/TR-200/woosh_navigation/Costmap/woosh_costmap/config/costmap_common_params.yaml`
- `src/TR-200/woosh_navigation/Costmap/woosh_costmap/config/global_costmap_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/move_base_only.launch`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/navigation.launch`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/move_base_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/global_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_costmap_params.yaml`
- `src/TR-200/woosh_slam/GMapping/woosh_slam_gmapping/launch/gmapping.launch`
- `src/TR-200/woosh_slam/Cartographer/woosh_slam_cartographer/launch/cartographer.launch`
- `src/TR-200/woosh_slam/Cartographer/woosh_slam_cartographer/launch/cartographer_localization.launch`
- `README.md`

### Core Constraints in the Current Architecture

1. `woosh_service_driver.py` is effectively the **single SDK owner** for the mobile base.
2. This project does **not** rely on spawning an additional external sensor bridge. Instead, `SmoothTwistController` inside the driver publishes `/scan`, `/odom`, `/odom_raw`, and TF directly.
3. Even when integrating with move_base, the system must **reuse the existing connection** rather than creating a second WebSocket owner. The `/cmd_vel` passthrough structure must be preserved.
4. Therefore, the new architecture must keep the **driver as the mobile core owner**, and use `launch_sensor_bridge:=false` where needed to avoid duplicate SDK connections.
5. Existing AMCL / GMapping / Cartographer launch files already expose arguments such as `launch_sensor_bridge` and `launch_rviz`, so they should be reused from the upper orchestration layer.
6. `move_base_only.launch` and `navigation.launch` currently fix the planner stack to `navfn + DWA`.
7. `cartographer_localization.launch` supports `fix / nonfix`, and in nonfix mode it separates `/map` into `/carto_map`.
8. `woosh_navigation_mb/CMakeLists.txt` likely needs improvement because its install rules appear incomplete.

---

## Final Goal

The refactored structure must support the following.

### 1. Single launch-file execution

Final entry point:

- `roslaunch woosh_bringup woosh_navigation_system.launch ...`

### 2. Algorithm selection only through launch arguments

#### Localization modes
- `none`
- `amcl`
- `carto_fix`
- `carto_nonfix`

#### SLAM modes
- `none`
- `gmapping`
- `cartographer`

#### Navigation modes
- `none`
- `costmap`
- `move_base`

### 3. Supported combinations

- localization only
- slam only
- localization + global costmap
- localization + move_base
- slam + global costmap
- slam + move_base

### 4. Planner swapping through launch args / YAML only

- Default global planner: `navfn`
- Default local planner: `DWA`
- However, without changing Python code, the following must be possible:
  - direct plugin class override
  - planner parameter YAML override
- In other words, the launch layer must support inputs such as:
  - `global_planner_plugin:=navfn/NavfnROS`
  - `local_planner_plugin:=dwa_local_planner/DWAPlannerROS`
  - `global_planner_params_file:=...`
  - `local_planner_params_file:=...`
- Additional planners such as `global_planner/GlobalPlanner` or `teb_local_planner/TebLocalPlannerROS` must **not** be enforced as required dependencies.
- If the user has those packages installed, the system should support them through plugin/path override only.

### 5. Backward compatibility with the existing CLI

The following legacy commands should continue to work if reasonably possible:

- `rosrun woosh_bringup woosh_service_driver.py amcl move_base_on ...`
- `rosrun woosh_bringup woosh_service_driver.py carto_loc_fix move_base_on ...`
- `rosrun woosh_bringup woosh_service_driver.py gmap`
- `rosrun woosh_bringup woosh_service_driver.py carto_map`

But internally, they should map into the new canonical parameter/mode structure.
That is:

- **CLI = legacy alias interface**
- **launch parameters = canonical interface**

---

## Preferred Design Direction

Adopt the following structure.

### A. Add a top-level launch file

Create a new file:

- `src/TR-200/woosh_bringup/launch/woosh_navigation_system.launch`

Requirements:

- This launch file must expose all major mode-selection and config options as launch args.
- It must launch `woosh_service_driver.py` as a node and pass all required parameters into it through ROS parameters.
- Do **not** directly include every child launch file from the top-level launch and try to manage ordering there.
- Instead, keep the current pattern where `woosh_service_driver.py` performs readiness checks and then spawns the required child launch files via subprocess.
- In short:
  - launch file = **configuration input**
  - driver = **runtime orchestrator**

### B. Refactor `woosh_service_driver.py`

- Change the **primary interface** from CLI-first to ROS-param-first.
- Example canonical parameters:
  - `~slam_mode` : `none|gmapping|cartographer`
  - `~localization_mode` : `none|amcl|carto_fix|carto_nonfix`
  - `~navigation_mode` : `none|costmap|move_base`
  - `~launch_rviz` : bool
  - `~rviz_config` : string (optional)
  - `~map_file`
  - `~state_file`
  - `~robot_ip`
  - `~robot_port`
  - `~global_planner_plugin`
  - `~local_planner_plugin`
  - `~global_planner_params_file`
  - `~local_planner_params_file`
  - `~move_base_params_file`
  - `~costmap_common_params_file`
  - `~global_costmap_params_file`
  - `~local_costmap_params_file`
  - `~load_global_planner_params`
  - `~load_local_planner_params`
  - `~auto_cmd_vel_passthrough`
- Preserve the CLI parser, but internally map it to canonical modes.
- Example legacy alias mappings:
  - `gmap -> gmapping`
  - `carto_map -> cartographer`
  - `carto_loc_fix -> carto_fix`
  - `carto_loc_nonfix -> carto_nonfix`
  - `nav_on -> costmap`
  - `move_base_on -> move_base`

### C. Refactor validation rules

Invalid combinations:

- `slam_mode != none` and `localization_mode != none`
- `navigation_mode != none` while both slam and localization are `none`

Valid combinations:

- `slam_mode != none` and `navigation_mode != none`
- `localization_mode != none` and `navigation_mode != none`

For invalid combinations, print a clear error message and terminate.

### D. Reuse child launch files

- Reuse the existing launch files as much as possible.
- When the driver spawns child launches, apply these principles:
  - use `launch_sensor_bridge:=false` by default
  - centralize RViz control through the top-level parameter surface
- Do not throw away the existing launch files completely. Keep them as reusable low-level launchers.

---

## Detailed Implementation Requirements

### 1) Create the new top-level launch file

File:

- `src/TR-200/woosh_bringup/launch/woosh_navigation_system.launch`

Required launch args:

- `robot_ip` (default: `169.254.128.2`)
- `robot_port` (default: `5480`)
- `slam_mode` (default: `none`)
- `localization_mode` (default: `none`)
- `navigation_mode` (default: `none`)
- `launch_rviz` (default: `true`)
- `rviz_config` (default: empty or auto)
- `map_file`
- `state_file`
- `global_planner_plugin` (default: `navfn/NavfnROS`)
- `local_planner_plugin` (default: `dwa_local_planner/DWAPlannerROS`)
- `global_planner_params_file`
- `local_planner_params_file`
- `move_base_params_file`
- `costmap_common_params_file`
- `global_costmap_params_file`
- `local_costmap_params_file`
- `load_global_planner_params` (default: `true`)
- `load_local_planner_params` (default: `true`)

These args must be forwarded into `woosh_service_driver.py` as ROS params.
The launch file must be a normal `roslaunch` entry point that can start everything without manually starting `roscore` first.

### 2) Refactor `woosh_service_driver.py` into a launch-param-centered orchestrator

Mandatory tasks:

- In `main()`, read canonical modes from ROS params first.
- If legacy CLI arguments are provided, translate them into canonical modes.
- Preserve `StackLauncher`, but extend or reorganize method signatures such as:
  - `start_move_base(...)`
  - `start_costmap(...)`
  - and if needed, normalize signatures for `start_amcl(...)` and `start_cartographer_localization(...)`
- Make planner/costmap file paths configurable through params.
- The existing `enable_cmd_vel_passthrough()` behavior that was tied to `move_base_on` must also trigger when canonical `navigation_mode == move_base`.
- Generalize readiness checks so they work for:
  - localization-based navigation startup
  - SLAM-based navigation startup
- Replace the old logic that said `nav_on` / `move_base_on` requires localization.
- The new rule should be: navigation requires **either SLAM or localization**.

### 3) Allow SLAM + navigation

The following must be explicitly supported:

- `gmapping + move_base`
- `cartographer + move_base`
- `gmapping + costmap`
- `cartographer + costmap`

Behavioral rules:

- In SLAM mode, navigation must consume the live `/map`.
- Therefore, in SLAM + navigation combinations, **do not launch an extra `map_server`**.
- `wait_for_nav_prerequisites()` must be generalized so that in SLAM mode it checks `/map` and the `map -> base_link` TF chain appropriately.

### 4) Strengthen the Cartographer localization + navigation path

This part is critical.

Because `cartographer_localization.launch` has both `fix` and `nonfix` behavior, map management for navigation can easily become ambiguous.
After the refactor, make the following behavior explicit:

- `carto_fix + costmap` : requires static `map_server`
- `carto_fix + move_base` : requires static `map_server`
- `carto_nonfix + costmap` : preserve current intent; costmap uses static `/map`, live map remains `/carto_map`
- `carto_nonfix + move_base` : explicitly decide which map topic feeds navigation, and implement that consistently

Recommended policy:

- In localization-based navigation, both costmap and move_base should use static `map_server(/map)`.
- In `carto_nonfix`, the live map should remain on `/carto_map` for RViz / diagnostics.

In other words, clearly separate:

- `slam_mode=cartographer` → live `/map`
- `localization_mode=carto_nonfix` → static `/map` + live `/carto_map`

### 5) Parameterize `move_base_only.launch`

File:

- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/move_base_only.launch`

Mandatory changes:

Replace current hard-coded values such as:

- `base_global_planner=navfn/NavfnROS`
- `base_local_planner=dwa_local_planner/DWAPlannerROS`

with arg-driven values.

Add launch args:

- `global_planner_plugin`
- `local_planner_plugin`
- `global_planner_params_file`
- `local_planner_params_file`
- `move_base_params_file`
- `costmap_common_params_file`
- `global_costmap_params_file`
- `local_costmap_params_file`
- `load_global_planner_params`
- `load_local_planner_params`

Planner YAML loading should be conditional via boolean args.
Default behavior must remain identical to the current `navfn + DWA` stack.
If custom plugin names or YAML paths are passed in, the planner stack must switch without source edits.

### 6) Make `global_costmap.launch` accept path injection via params

File:

- `src/TR-200/woosh_navigation/Costmap/woosh_costmap/launch/global_costmap.launch`

Required changes:

- If YAML paths are currently hard-coded, refactor them into launch args.
- Add args such as:
  - `costmap_common_params_file`
  - `global_costmap_params_file`
  - `launch_map_server`
  - `launch_sensor_bridge`
  - `launch_map_odom_tf`
  - `launch_base_laser_tf`
- In SLAM + costmap mode, it must be possible to use `launch_map_server:=false`.

### 7) Remove duplicated orchestration in `navigation.launch`

File:

- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/navigation.launch`

Requirements:

- If this file currently duplicates full-stack orchestration, align it with the new canonical structure.
- Preferred direction:
  - turn `navigation.launch` into a thin wrapper or deprecated wrapper
  - make `woosh_bringup/launch/woosh_navigation_system.launch` the canonical entry point
- Avoid having two separate sources of truth for stack orchestration.

### 8) Clean up CMake / package metadata

Must review and update:

- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/CMakeLists.txt`
  - add `install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})`
  - if needed, align `find_package(catkin REQUIRED COMPONENTS ...)` with `package.xml`
- `src/TR-200/woosh_bringup/CMakeLists.txt`
  - if a new config directory is added, include it in install targets
- update `package.xml` only when a truly required dependency is introduced
- do **not** force planners such as `global_planner` or `teb_local_planner` as required dependencies; keep them as optional override paths

### 9) Update the README

File:

- `README.md`

Required changes:

- Keep the old multi-CLI usage notes as legacy usage.
- Add the new canonical launch-based workflow near the top.
- Include at least the following 5 examples:
  1. AMCL + move_base
  2. Cartographer fix localization + move_base
  3. GMapping + move_base
  4. Cartographer SLAM + move_base
  5. AMCL + move_base + custom planner plugin / YAML
- Write the example commands using the actual repo structure.

---

## Canonical Execution Examples That the Final Refactor Must Support

### 1. AMCL + move_base

```bash
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl \
  navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml
```

### 2. Cartographer fix localization + move_base

```bash
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=carto_fix \
  navigation_mode:=move_base \
  state_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml
```

### 3. GMapping + move_base

```bash
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=gmapping \
  navigation_mode:=move_base
```

### 4. Cartographer SLAM + move_base

```bash
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=cartographer \
  navigation_mode:=move_base
```

### 5. AMCL + move_base + custom planner

```bash
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl \
  navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml \
  global_planner_plugin:=global_planner/GlobalPlanner \
  global_planner_params_file:=/abs/path/to/global_planner.yaml \
  local_planner_plugin:=dwa_local_planner/DWAPlannerROS \
  local_planner_params_file:=/root/catkin_ws/src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml
```

---

## Required Output Format

Your implementation response must be organized in the following order:

1. Changed file list
2. Change summary (1–3 lines per file)
3. Actual modified code
   - preferably in unified diff format
   - otherwise full file contents per file
4. Final execution examples
5. Validation checklist
   - expected topics / TF by mode combination
   - catkin build check points
   - backward compatibility check points for legacy CLI

---

## Prohibited

Do **not** do any of the following:

- provide only an abstract design memo
- give only explanatory guidance without code
- migrate to ROS2 / Nav2
- break the single-SDK-owner structure by spawning duplicate sensor bridges or duplicate WebSocket owners
- design the system such that planner swapping still requires Python source modification each time
