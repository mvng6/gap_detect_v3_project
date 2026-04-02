# Refactoring Plan Spec: Launch-Parameter-Centered Navigation Orchestration

> Authored: 2026-04-02
> Based on: `01_Refactoring_prompt.md` + deep code review + 15-question interview with maintainer
> Status: FINALIZED — ready for phased implementation

---

## 1. Clarified Assumptions

| Topic | Assumption |
|-------|------------|
| **SDK ownership** | `woosh_service_driver.py` is the sole WebSocket owner. `bringup.launch` and `woosh_sensor_bridge.py` are standalone fallbacks. They are NOT the production path when the driver is running. |
| **Sensor publishing** | `SmoothTwistController` (inside driver) publishes `/scan`, `/odom`, `/odom_raw`, TF(`odom→base_link`). These are identical in frame semantics to what `woosh_sensor_bridge.py` would publish (same frame IDs: `laser`, `odom`, `base_link`). |
| **`base_link→laser` TF** | Not needed in manual-only mode, but required for RViz, SLAM, localization, and navigation. Currently only published by SLAM/localization child launch files. The refactor must make ownership explicit and guaranteed. |
| **`bringup.launch`** | Treated as legacy/fallback. Not the canonical production path. Not modified, not removed. |
| **`navigation.launch`** | Legacy self-contained stack. Nobody should call it alongside the driver. To be deprecated as a thin wrapper pointing to `woosh_navigation_system.launch`. |
| **SLAM + navigation block** | Was NOT a proven technical impossibility. It was an incomplete implementation. We will unblock it with conservative readiness checks. |
| **`carto_nonfix` + navigation** | Requires BOTH `state_file:=` (for cartographer) AND `map_file:=` (for static `map_server`). These must correspond to the same map session. Mismatched sources are not a supported workflow. |
| **Readiness timeout values** | No production data proving current defaults are optimal. Keep reasonable defaults; make them configurable via ROS params. Improve diagnostic logs. |
| **Node name** | `mobile_move_server` is kept unchanged. `/mobile_move` service must remain at its absolute path. |
| **Planner injection** | Via subprocess launch args only. Not via pre-seeded ROS params. The config surface must be explicit and launch-driven. |
| **Deployment paths** | `/root/catkin_ws/...` paths are valid in production Docker container. Prefer `$(find ...)` / `rospkg` lookups in launch files over hardcoded absolute paths in Python. |
| **RViz config** | No new all-in-one RViz config file required in Phase 1. Support `rviz_config` override; default to best matching existing config. |
| **Test/validation** | Primarily manual. "It works" = clean launch, required topics/TF become active, robot executes at least one 2D Nav Goal in RViz. |
| **`global_costmap.launch`** | Already has `launch_map_server`, `launch_sensor_bridge`, `launch_map_odom_tf`, `launch_base_laser_tf` args. **Missing**: `costmap_common_params_file` and `global_costmap_params_file` args — YAML paths are currently hardcoded to `$(find woosh_costmap)/config/...`. Must add these. |

---

## 2. Final Architecture Decisions

### A. Entry-point hierarchy

```
roslaunch woosh_bringup woosh_navigation_system.launch [args]
  └─ launches: woosh_service_driver.py  (as ROS node)
                  ├─ reads: ~slam_mode, ~localization_mode, ~navigation_mode, ~*, etc.
                  ├─ spawns (subprocess): gmapping.launch | cartographer.launch
                  ├─ spawns (subprocess): amcl.launch | cartographer_localization.launch
                  ├─ spawns (subprocess): global_costmap.launch | move_base_only.launch
                  └─ spawns (subprocess): rviz [optional]
```

- `roslaunch` is the **configuration surface**.
- `woosh_service_driver.py` remains the **runtime orchestrator** (readiness checks, SDK owner, subprocess lifecycle).
- Child launches are reused as low-level launchers. All called with `launch_sensor_bridge:=false` and `launch_rviz:=false` (driver controls these).

### B. `base_link → laser` TF ownership policy

| Mode | Who publishes `base_link→laser` |
|------|----------------------------------|
| Basic (no SLAM/loc/nav) | `StackLauncher.start_base_laser_tf()` — new method, called by driver |
| SLAM (`gmapping`, `cartographer`) | Child launch file — already has `static_transform_publisher` |
| Localization (`amcl`, `carto_fix`, `carto_nonfix`) | Child launch file — already has static TF |
| SLAM + navigation / Localization + navigation | Child launch file (SLAM/loc) — costmap launch called with `launch_base_laser_tf:=false` |

The `woosh_navigation_system.launch` will NOT publish this TF itself; the driver manages it programmatically.

### C. Map topic policy per mode

| Mode | `/map` source | Navigation uses |
|------|--------------|----------------|
| `slam=gmapping` | GMapping live output | live `/map` — NO `map_server` |
| `slam=cartographer` | `cartographer_occupancy_grid_node` live output | live `/map` — NO `map_server` |
| `loc=amcl` | `amcl.launch` → `map_server` | static `/map` from `map_server` |
| `loc=carto_fix` | `cartographer_localization.launch` (fix mode, no `/map` output) | static `/map` from `map_server` required |
| `loc=carto_nonfix` | `map_server` (static `/map`) + `cartographer_occupancy_grid_node` (live `/carto_map`) | static `/map` — `/carto_map` for diagnostics only |

For SLAM + navigation: `start_costmap()` and `start_move_base()` are called with `launch_map_server:=false`.

### D. Validation rule rewrite

**Invalid combinations (terminate with clear error):**
1. `slam_mode != none AND localization_mode != none` — cannot run SLAM and localization simultaneously
2. `navigation_mode != none AND slam_mode == none AND localization_mode == none` — navigation requires a map source

**Valid combinations (previously blocked, now supported):**
- `slam_mode != none AND navigation_mode != none`
- `localization_mode != none AND navigation_mode != none`

### E. Readiness check generalization

Current `wait_for_nav_prerequisites()` only handles localization-sourced `map→odom` TF. Must generalize:

```
In SLAM mode:
  - wait for /map topic (SLAM node publishes it after first loop closure / submap)
  - wait for TF: map → odom (SLAM publishes this)
  - wait for TF: odom → base_link (SmoothTwistController publishes this)

In localization mode:
  - wait for /map topic (map_server — usually immediate)
  - wait for TF: map → odom (AMCL or cartographer — can take 10-15s)
  - wait for TF: odom → base_link (SmoothTwistController — already ready)
```

Timeout values: expose via ROS params `~nav_prerequisites_timeout` (default: 30.0) and `~costmap_ready_timeout` (default: 20.0).

### F. Planner injection

`StackLauncher.start_move_base()` constructs subprocess args explicitly:

```python
cmd = [
    'roslaunch', 'woosh_navigation_mb', 'move_base_only.launch',
    f'global_planner_plugin:={global_planner_plugin}',
    f'local_planner_plugin:={local_planner_plugin}',
    f'move_base_params_file:={move_base_params_file}',
    f'costmap_common_params_file:={costmap_common_params_file}',
    f'global_costmap_params_file:={global_costmap_params_file}',
    f'local_costmap_params_file:={local_costmap_params_file}',
    f'global_planner_params_file:={global_planner_params_file}',
    f'local_planner_params_file:={local_planner_params_file}',
    f'load_global_planner_params:={load_global_planner_params}',
    f'load_local_planner_params:={load_local_planner_params}',
]
```

### G. CLI backward compatibility

CLI args are translated into canonical mode strings **after** ROS param reading. Priority: ROS params first, CLI args override if set.

| Legacy CLI flag | Canonical mapping |
|-----------------|-------------------|
| `gmap` | `slam_mode=gmapping` |
| `carto_map` | `slam_mode=cartographer` |
| `amcl` | `localization_mode=amcl` |
| `carto_loc_fix` | `localization_mode=carto_fix` |
| `carto_loc_nonfix` | `localization_mode=carto_nonfix` |
| `nav_on` / `costmap` | `navigation_mode=costmap` |
| `move_base_on` | `navigation_mode=move_base` |
| `rviz_on` | `launch_rviz=true` |

---

## 3. File-Level Refactoring Scope

### New files

| File | Purpose |
|------|---------|
| `src/TR-200/woosh_bringup/launch/woosh_navigation_system.launch` | Single canonical entry point. Exposes all mode/config args. Launches driver node with params. |

### Modified files

| File | Scope of changes |
|------|-----------------|
| `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py` | Param-first `main()`, canonical mode enum, SLAM+nav combinations, generalized readiness checks, `start_base_laser_tf()`, extended `start_move_base()` / `start_costmap()` signatures |
| `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/move_base_only.launch` | Add `global_planner_plugin`, `local_planner_plugin`, `*_params_file`, `load_*_params` args. Keep navfn+DWA as defaults. |
| `src/TR-200/woosh_navigation/Costmap/woosh_costmap/launch/global_costmap.launch` | Add `costmap_common_params_file` and `global_costmap_params_file` args. YAML paths currently hardcoded. |
| `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/launch/navigation.launch` | Add deprecation header comment. Keep file intact as legacy wrapper. |
| `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/CMakeLists.txt` | Add `install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})` |
| `README.md` | Add canonical launch examples near top. Keep legacy CLI section as-is. |

### Unchanged files (verified, no action needed)

- `amcl.launch` — already has `launch_sensor_bridge`, `launch_rviz`, `launch_base_laser_tf` args
- `gmapping.launch` — already has `launch_sensor_bridge`, `launch_rviz` args
- `cartographer.launch` — already has `launch_sensor_bridge`, `launch_rviz` args
- `cartographer_localization.launch` — already handles fix/nonfix and `/carto_map` remap
- `global_costmap.launch` — only needs YAML path args added (see above)
- `static_tf.launch` — unchanged, remains as standalone utility
- `bringup.launch` — unchanged, remains as legacy fallback
- `woosh_sensor_bridge.py` — unchanged
- All `config/*.yaml` files — unchanged (referenced by path from launch/driver)

---

## 4. Launch / Parameter Design

### `woosh_navigation_system.launch` — full arg list

```xml
<!-- Robot connection -->
<arg name="robot_ip"                   default="169.254.128.2" />
<arg name="robot_port"                 default="5480" />
<arg name="robot_identity"             default="navigation_system" />

<!-- Mode selection -->
<arg name="slam_mode"                  default="none" />   <!-- none|gmapping|cartographer -->
<arg name="localization_mode"          default="none" />   <!-- none|amcl|carto_fix|carto_nonfix -->
<arg name="navigation_mode"            default="none" />   <!-- none|costmap|move_base -->

<!-- Map sources (required when localization or carto_nonfix+nav) -->
<arg name="map_file"                   default="" />       <!-- .yaml for map_server -->
<arg name="state_file"                 default="" />       <!-- .pbstream for cartographer loc -->

<!-- Visualization -->
<arg name="launch_rviz"               default="true" />
<arg name="rviz_config"               default="" />        <!-- empty = auto-select -->

<!-- Planner selection (move_base mode only) -->
<arg name="global_planner_plugin"      default="navfn/NavfnROS" />
<arg name="local_planner_plugin"       default="dwa_local_planner/DWAPlannerROS" />

<!-- Planner/costmap YAML override paths -->
<arg name="move_base_params_file"      default="$(find woosh_navigation_mb)/config/move_base_params.yaml" />
<arg name="costmap_common_params_file" default="$(find woosh_costmap)/config/costmap_common_params.yaml" />
<arg name="global_costmap_params_file" default="$(find woosh_costmap)/config/global_costmap_params.yaml" />
<arg name="local_costmap_params_file"  default="$(find woosh_navigation_mb)/config/local_costmap_params.yaml" />
<arg name="global_planner_params_file" default="$(find woosh_navigation_mb)/config/global_planner_params.yaml" />
<arg name="local_planner_params_file"  default="$(find woosh_navigation_mb)/config/local_planner_params.yaml" />
<arg name="load_global_planner_params" default="true" />
<arg name="load_local_planner_params"  default="true" />

<!-- Timeout overrides -->
<arg name="nav_prerequisites_timeout"  default="30.0" />
<arg name="costmap_ready_timeout"      default="20.0" />
```

All args are forwarded to `mobile_move_server` node as `<param>` elements under the node's private namespace (`~`).

### `woosh_service_driver.py` — new `main()` param reading order

```python
def main():
    # 1. Parse CLI argv (before rospy.init_node, since it strips ROS args)
    flags, cli_map_file, cli_state_file, filtered_argv = _parse_cli_args(argv)

    # 2. Init ROS node
    rospy.init_node('mobile_move_server', argv=filtered_argv, anonymous=False)

    # 3. Read canonical mode from ROS params (set by roslaunch)
    slam_mode         = rospy.get_param('~slam_mode', 'none').lower()
    localization_mode = rospy.get_param('~localization_mode', 'none').lower()
    navigation_mode   = rospy.get_param('~navigation_mode', 'none').lower()
    launch_rviz       = rospy.get_param('~launch_rviz', False)
    rviz_config       = rospy.get_param('~rviz_config', '')
    map_file          = rospy.get_param('~map_file', cli_map_file or '')
    state_file        = rospy.get_param('~state_file', cli_state_file or '')
    # ... planner params, timeout params ...

    # 4. If CLI flags are present, they override/supplement ROS params
    #    (legacy alias translation happens here)
    if flags.get('gmap'):         slam_mode         = 'gmapping'
    if flags.get('carto_map'):    slam_mode         = 'cartographer'
    if flags.get('amcl'):         localization_mode = 'amcl'
    if flags.get('carto_loc_fix'):   localization_mode = 'carto_fix'
    if flags.get('carto_loc_nonfix'): localization_mode = 'carto_nonfix'
    if flags.get('nav_on') or flags.get('costmap'):   navigation_mode = 'costmap'
    if flags.get('move_base_on'):  navigation_mode  = 'move_base'
    if flags.get('rviz_on'):       launch_rviz      = True

    # 5. Validate combinations
    _validate_modes(slam_mode, localization_mode, navigation_mode)
    # ...
```

### `move_base_only.launch` — new arg surface (key additions)

```xml
<arg name="global_planner_plugin"      default="navfn/NavfnROS" />
<arg name="local_planner_plugin"       default="dwa_local_planner/DWAPlannerROS" />
<arg name="move_base_params_file"      default="$(find woosh_navigation_mb)/config/move_base_params.yaml" />
<arg name="costmap_common_params_file" default="$(find woosh_costmap)/config/costmap_common_params.yaml" />
<arg name="global_costmap_params_file" default="$(find woosh_costmap)/config/global_costmap_params.yaml" />
<arg name="local_costmap_params_file"  default="$(find woosh_navigation_mb)/config/local_costmap_params.yaml" />
<arg name="global_planner_params_file" default="$(find woosh_navigation_mb)/config/global_planner_params.yaml" />
<arg name="local_planner_params_file"  default="$(find woosh_navigation_mb)/config/local_planner_params.yaml" />
<arg name="load_global_planner_params" default="true" />
<arg name="load_local_planner_params"  default="true" />
```

YAML loading becomes conditional:
```xml
<rosparam if="$(arg load_global_planner_params)"
          file="$(arg global_planner_params_file)" command="load"
          ns="move_base/$(arg global_planner_plugin)" />
```

Plugin args are injected via `<param>`:
```xml
<param name="move_base/base_global_planner" value="$(arg global_planner_plugin)" />
<param name="move_base/base_local_planner"  value="$(arg local_planner_plugin)" />
```

### `global_costmap.launch` — additions

Add two new args with `$(find woosh_costmap)` defaults:
```xml
<arg name="costmap_common_params_file"
     default="$(find woosh_costmap)/config/costmap_common_params.yaml" />
<arg name="global_costmap_params_file"
     default="$(find woosh_costmap)/config/global_costmap_params.yaml" />
```

Replace hardcoded `<rosparam file="$(find woosh_costmap)/config/...">` with:
```xml
<rosparam file="$(arg costmap_common_params_file)" command="load" ns="costmap" />
<rosparam file="$(arg global_costmap_params_file)" command="load" ns="costmap" />
```

---

## 5. Compatibility Strategy

### Legacy CLI — must continue working (high priority)

The following `rosrun` invocations are high priority and must continue working after the refactor:

```bash
rosrun woosh_bringup woosh_service_driver.py gmap
rosrun woosh_bringup woosh_service_driver.py carto_map
rosrun woosh_bringup woosh_service_driver.py carto_loc_nonfix
rosrun woosh_bringup woosh_service_driver.py amcl nav_on map_file:=...
rosrun woosh_bringup woosh_service_driver.py carto_loc_fix nav_on state_file:=... map_file:=...
rosrun woosh_bringup woosh_service_driver.py carto_loc_nonfix nav_on state_file:=... map_file:=...
rosrun woosh_bringup woosh_service_driver.py amcl move_base_on map_file:=...
rosrun woosh_bringup woosh_service_driver.py carto_loc_fix move_base_on state_file:=... map_file:=...
```

Lower priority (best-effort):
```bash
rosrun woosh_bringup woosh_service_driver.py carto_loc_nonfix move_base_on state_file:=... map_file:=...
```

Implementation: the `_parse_cli_args()` function is preserved exactly. The translated canonical modes are passed to the same `StackLauncher` methods. The only internal change is the mode handling logic in `main()`.

### New canonical examples (must work post-refactor)

```bash
# 1. AMCL + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# 2. Cartographer fix + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=carto_fix navigation_mode:=move_base \
  state_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# 3. GMapping + move_base (SLAM+nav — previously blocked)
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=gmapping navigation_mode:=move_base

# 4. Cartographer SLAM + move_base (SLAM+nav — previously blocked)
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=cartographer navigation_mode:=move_base

# 5. AMCL + move_base + custom planner
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml \
  global_planner_plugin:=global_planner/GlobalPlanner \
  global_planner_params_file:=/abs/path/to/global_planner.yaml
```

---

## 6. Validation Strategy

### By mode combination — expected topics and TF after startup

| Mode | Required topics | Required TF chain |
|------|----------------|-------------------|
| Driver only (basic) | `/scan`, `/odom`, `/odom_raw` | `odom→base_link`, `base_link→laser` |
| `slam=gmapping` | + `/map` | + `map→odom` |
| `slam=cartographer` | + `/map` | + `map→odom` |
| `loc=amcl` | + `/map`, `/amcl/parameter_descriptions` | + `map→odom` |
| `loc=carto_fix` | `/map` from map_server | + `map→odom` |
| `loc=carto_nonfix` | `/map` (map_server) + `/carto_map` | + `map→odom` |
| `+ nav=costmap` | + `/global_costmap/costmap` | same as above |
| `+ nav=move_base` | + `/move_base/status`, `/cmd_vel` published | same as above |

### Manual validation sequence

1. `catkin build woosh_bringup woosh_navigation_mb woosh_costmap` — must build cleanly
2. `roslaunch woosh_bringup woosh_navigation_system.launch localization_mode:=amcl navigation_mode:=move_base map_file:=...` — must launch without ERROR logs
3. `rostopic echo /global_costmap/costmap --once` — must receive a message
4. `rostopic echo /move_base/status --once` — must receive a GoalStatusArray
5. In RViz: click "2D Nav Goal" → robot must accept goal and publish `/cmd_vel`
6. Legacy CLI: `rosrun woosh_bringup woosh_service_driver.py amcl move_base_on map_file:=...` — must behave identically to above

### Backward-compatibility checks

- Service `/mobile_move` must remain available in all modes
- Node name `mobile_move_server` must be unchanged
- CLI flags `gmap`, `carto_map`, `amcl`, `carto_loc_fix`, `carto_loc_nonfix`, `nav_on`, `move_base_on`, `rviz_on` must all still be accepted
- `map_file:=` and `state_file:=` CLI kwargs must still be parsed correctly

---

## 7. Risks, Open Decisions, and Mitigations

### Risk 1: SLAM startup timing before navigation

**Problem**: In SLAM + move_base, `/map` and `map→odom` TF may not be available until the SLAM node has processed several scans (GMapping) or built a first submap (Cartographer). move_base starting before this point will crash or fail to initialize the global costmap.

**Mitigation**: `wait_for_nav_prerequisites()` already waits for `/map` and `map→odom` TF. In SLAM mode, increase the default timeout via `~nav_prerequisites_timeout` (default: 30.0s). Add a diagnostic log line when waiting for SLAM to produce its first map. The driver should only call `start_costmap()` / `start_move_base()` AFTER prerequisites are confirmed.

### Risk 2: `base_link→laser` TF missing in basic mode

**Problem**: When driver runs without SLAM/localization, no child launch file publishes `base_link→laser`. RViz won't display `/scan` correctly.

**Mitigation**: Add `StackLauncher.start_base_laser_tf()` method that starts a `static_transform_publisher` subprocess. Called in `main()` when SLAM/localization are both `none` (basic mode). Not called otherwise (SLAM/loc child launches own it). The top-level `woosh_navigation_system.launch` does NOT publish this TF directly — the driver manages it programmatically to maintain ownership clarity.

### Risk 3: `carto_nonfix` + navigation map source ambiguity

**Problem**: If both `state_file` and `map_file` are provided for `carto_nonfix + move_base`, the static `map_server(/map)` and live `cartographer(/carto_map)` must be clearly separated. If someone passes only `state_file` (no `map_file`), navigation has no static map.

**Mitigation**: Validate in `main()` that when `localization_mode=carto_nonfix` AND `navigation_mode != none`, both `map_file` and `state_file` must be non-empty. Print a clear error and terminate if missing.

### Risk 4: `navigation.launch` being called by external tools

**Problem**: If something outside the visible codebase calls `navigation.launch` directly, turning it into a thin wrapper that delegates to `woosh_navigation_system.launch` could change behavior.

**Mitigation**: Keep `navigation.launch` intact. Add only a header deprecation comment. Do not change its functional behavior. The wrapper approach is a future step after confirming no external dependencies.

### Risk 5: Subprocess ROS arg escaping

**Problem**: Path strings in `start_move_base()` subprocess call may contain characters that break shell parsing if paths have spaces.

**Mitigation**: Use `subprocess.Popen(cmd_list)` (not `shell=True`) with a proper list. No string joining. Already the current pattern in `SubprocessManager.start()`.

### Risk 6: `load_global_planner_params` + YAML namespace

**Problem**: `<rosparam if="$(arg load_global_planner_params)" file="$(arg global_planner_params_file)" ns="move_base/$(arg global_planner_plugin)">` — the namespace uses the plugin string which may contain `/` (e.g., `navfn/NavfnROS`). ROS param namespaces with slashes create nested hierarchy, which is expected but must be verified.

**Mitigation**: Test with default navfn plugin. The current `global_planner_params.yaml` loads under `move_base/NavfnROS/...` which matches how navfn expects it. Document the expected namespace in `move_base_only.launch` comments.

---

## 8. Phased Implementation Order

### Phase 1 — Parameterize low-level launch files
*Goal: No behavior changes. Just add args to child launches so Phase 2 can inject them.*

1. **`move_base_only.launch`**: Add `global_planner_plugin`, `local_planner_plugin`, all `*_params_file` and `load_*_params` args. Defaults preserve current navfn+DWA behavior exactly.
2. **`global_costmap.launch`**: Add `costmap_common_params_file` and `global_costmap_params_file` args. Defaults preserve current behavior.
3. **`navigation.launch`**: Add deprecation comment block at top. No functional changes.
4. **`woosh_navigation_mb/CMakeLists.txt`**: Add install targets for `launch/` and `config/`.

**Validation**: `catkin build` succeeds. All existing `rosrun` workflows unchanged.

---

### Phase 2 — Create canonical launch entry point + param-first driver interface
*Goal: New `roslaunch` path works. Legacy `rosrun` path still works unchanged.*

5. **`woosh_navigation_system.launch`**: Create new file. Expose all mode/config args. Launch `mobile_move_server` node with `<param>` forwarding.
6. **`woosh_service_driver.py` — `main()` refactor**:
   - Add canonical mode reading from `rospy.get_param('~slam_mode', 'none')` etc. at top of `main()`
   - Add legacy CLI → canonical mode translation block
   - Rewrite validation logic (`_validate_modes()`)
   - Read all planner/costmap param paths from ROS params
   - Preserve `_parse_cli_args()` exactly — just add translation layer after it

**Validation**: Canonical launch examples 1 and 2 (AMCL + move_base, carto_fix + move_base) work. All legacy CLI commands work.

---

### Phase 3 — SLAM + navigation support
*Goal: Combinations 3 and 4 (SLAM + move_base) work.*

7. **`woosh_service_driver.py` — SLAM+nav logic**:
   - Remove the validation block that rejects `nav_on`/`move_base_on` with SLAM flags
   - Generalize `wait_for_nav_prerequisites()` to accept `slam_mode` parameter
   - Add SLAM-aware costmap call: `start_costmap(launch_map_server=False)` in SLAM mode
   - Add `start_base_laser_tf()` to `StackLauncher` for basic mode
   - Extend `StackLauncher.start_move_base()` to accept and pass all planner args

**Validation**: Canonical launch examples 3 and 4 (GMapping + move_base, Cartographer + move_base) work.

---

### Phase 4 — Documentation and cleanup
*Goal: README up to date. All canonical examples documented.*

8. **`README.md`**: Add "Canonical launch workflow" section near top with all 5 examples. Keep existing CLI section as "Legacy CLI". Update Quick Start terminal 3 to reference `woosh_navigation_system.launch`.

---

## Appendix: Key Code Locations (verified)

| What | File | Line range |
|------|------|-----------|
| CLI arg parsing | `woosh_service_driver.py` | ~1820–1867 |
| Validation block (to rewrite) | `woosh_service_driver.py` | in `main()` ~2030–2060 |
| `StackLauncher.start_move_base()` | `woosh_service_driver.py` | ~760–810 |
| `StackLauncher.start_costmap()` | `woosh_service_driver.py` | ~660–720 |
| `wait_for_nav_prerequisites()` | `woosh_service_driver.py` | ~720–760 |
| `enable_cmd_vel_passthrough()` | `woosh_service_driver.py` (SmoothTwistController) | ~1400–1430 |
| Hardcoded YAML paths | `global_costmap.launch` | lines 132–136 |
| Hardcoded planner plugins | `move_base_only.launch` | (lines to verify during impl) |
