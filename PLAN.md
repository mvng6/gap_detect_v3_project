# Woosh TR-200 Navigation 전체 아키텍처 설계 계획

**프로젝트**: gap_detect_v3_project
**작성일**: 2026-03-24
**담당자**: LDJ @ KATECH
**대상 환경**: Ubuntu 24.04 host + Docker ROS Noetic
**로봇**: Woosh TR-200 (모바일) + Doosan a0912 (협동 암)

---

## [1] 전체 아키텍처 설계

### 1.1 왜 "Docker ROS 유지 + SDK Bridge" 방식인가

| 비교 항목 | 로봇 내부 ROS Master 전환 | Docker ROS + SDK Bridge (채택) |
|-----------|--------------------------|-------------------------------|
| 의존성 | 로봇 내부 ROS 버전/설정에 종속 | Docker 내 ROS를 완전 제어 가능 |
| 네트워크 | ROS_MASTER_URI 충돌 위험 | WebSocket 단일 포트만 사용 |
| 안정성 | 로봇 내부 상태 변경 시 전체 시스템 영향 | 로봇 내부와 독립적 |
| 확장성 | Doosan 암과의 통합 시 Master 충돌 | Docker 내 단일 Master로 통합 |
| 유지보수 | 로봇 펌웨어 업데이트 시 ROS 구조 깨짐 위험 | SDK 변경만 woosh_robot.py에 반영 |
| 개발 환경 | 로봇 없이 개발 어려움 | mock SDK로 오프라인 개발 가능 |

**결론**: Docker 내부 ROS Master를 기준으로, Woosh SDK를 adapter 계층으로 래핑하는 방식이 현재 프로젝트 구조와 일치하며 확장성이 뛰어나다.

### 1.2 전체 노드 구성 (목표 상태)

```
┌─────────────────────────────────────────────────────────────────┐
│  Docker Container (ROS Noetic Master)                           │
│                                                                  │
│  ┌──────────────────┐    ┌──────────────────────────────────┐   │
│  │  woosh_sensor_   │    │  robot_localization              │   │
│  │  bridge.py       │    │  (ekf_localization)              │   │
│  │                  │    │                                  │   │
│  │  /odom_raw ──────┼────┼→ /odom (fused)                  │   │
│  │  /scan           │    │  TF: odom → base_link            │   │
│  │  TF(odom→base)   │    └──────────────────────────────────┘   │
│  └──────────────────┘                                            │
│                                                                  │
│  ┌──────────────────┐    ┌──────────────────────────────────┐   │
│  │  cmd_vel_        │    │  SLAM / Localization              │   │
│  │  adapter.py      │    │  (gmapping / cartographer / amcl) │   │
│  │                  │    │                                  │   │
│  │  /cmd_vel ───────┼────┼→ SDK twist_req()                 │   │
│  │  (sub)           │    │  /map                            │   │
│  └──────────────────┘    │  TF: map → odom                  │   │
│                          └──────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  move_base                                               │   │
│  │  /cmd_vel (pub) → cmd_vel_adapter                        │   │
│  │  global_costmap / local_costmap                          │   │
│  │  /move_base/goal (actionlib)                             │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         │ WebSocket (ws://169.254.128.2:5480)
         ↓
┌─────────────────────────────┐
│  Woosh TR-200 Robot         │
│  PoseSpeed, ScannerData     │
│  twist_req() commands       │
└─────────────────────────────┘
```

### 1.3 토픽 흐름

```
SDK PoseSpeed.pose ──→ woosh_sensor_bridge ──→ /odom_raw (nav_msgs/Odometry)
                                           ──→ TF: odom → base_link (raw, 옵션)
SDK ScannerData    ──→ woosh_sensor_bridge ──→ /scan (sensor_msgs/LaserScan)

/odom_raw          ──→ robot_localization  ──→ /odom (nav_msgs/Odometry, fused)
                                           ──→ TF: odom → base_link (EKF fused)

/cmd_vel           ──→ cmd_vel_adapter     ──→ SDK twist_req()

/odom + /scan + /map ──→ move_base        ──→ /cmd_vel
move_base          ──→ global/local costmap ──→ /move_base/goal
```

### 1.4 TF 트리 구조

**SLAM/Navigation 동작 시 (목표)**:
```
map
└── odom  [SLAM 또는 AMCL이 발행]
    └── base_link  [robot_localization EKF 발행]
        └── laser  [static_transform_publisher, z=0.25m]
```

**현재 구조 (bringup only)**:
```
odom  [woosh_sensor_bridge 발행]
└── base_link  [woosh_sensor_bridge 발행]
    └── laser  [static_transform_publisher, z=0.25m]
```

> **주의**: `robot_localization`이 odom→base_link TF를 발행하므로,
> `woosh_sensor_bridge`의 TF 발행은 `publish_tf: false`로 비활성화해야 중복 방지 가능.

### 1.5 /odom_raw vs /odom 분리 전략

| 토픽 | 발행 노드 | 소스 | 용도 |
|------|----------|------|------|
| `/odom_raw` | woosh_sensor_bridge.py | SDK PoseSpeed.pose | EKF 입력, 원시 데이터 |
| `/odom` | robot_localization (ekf_node) | /odom_raw 융합 | Navigation/SLAM 입력 |

- **현재 상태**: woosh_sensor_bridge가 `/odom`을 직접 발행 중
- **목표 상태**: `/odom_raw`로 이름 변경 후 EKF를 통해 `/odom` 발행
- **마이그레이션**: 기존 `/odom` 의존 패키지(woosh_service_driver 등) 업데이트 필요

### 1.6 IMU Fallback 전략

**현황**: Woosh SDK protobuf에 IMU 관련 필드 없음 (IMU 데이터 SDK 미제공 확인됨)

| 시나리오 | 전략 |
|----------|------|
| IMU 없음 (현재) | EKF 없이 /odom_raw → /odom 직접 passthrough 또는 단순 EKF |
| IMU 있음 (향후) | robot_localization ekf_node로 /odom_raw + /imu 융합 |
| pseudo IMU | /cmd_vel + odom으로 yaw rate 추정 (정밀도 낮음, 비권장) |

**IMU 없을 때 EKF 설정**: odom_raw 단독 입력으로 스무딩 효과만 활용
→ 2D 모드, process_noise_covariance 적절히 설정

### 1.7 /cmd_vel → SDK 변환

```
ROS topic /cmd_vel (geometry_msgs/Twist)
    linear.x  → SDK Twist.linear  [m/s]
    angular.z → SDK Twist.angular [rad/s]
    ↓
cmd_vel_adapter.py
    ↓
await robot.twist_req(Twist(linear=vx, angular=wz))
```

**주의사항**:
- 최대 속도 제한 (현재 0.12 m/s) 적용
- 로봇 연결 끊김 시 zero 명령 전송
- /cmd_vel 토픽 타임아웃 시 자동 정지 (watchdog timer)

### 1.8 SLAM vs Navigation 단계 분리

```
[SLAM 단계]
woosh_sensor_bridge → /scan, /odom_raw
robot_localization → /odom
gmapping/cartographer → /map, TF(map→odom)
→ 맵 저장: rosrun map_server map_saver

[Navigation 단계]
map_server → /map (저장된 맵 서빙)
amcl → TF(map→odom) (위치추정)
move_base → /cmd_vel
cmd_vel_adapter → SDK twist
```

두 단계는 exclusive: SLAM 중에는 map_server 없음, navigation 중에는 SLAM 없음.

---

## [2] 패키지 구조 제안

### 2.1 현재 존재하는 패키지 (유지/수정)

| 패키지 | 경로 | 현재 상태 | 변경 필요 |
|--------|------|----------|----------|
| woosh_robot_py | src/TR-200/woosh_robot_py/ | 완성 | 최소 변경 |
| woosh_sensor_bridge | src/TR-200/woosh_sensor_bridge/ | /odom 발행 | /odom_raw로 변경 |
| woosh_bringup | src/TR-200/woosh_bringup/ | /mobile_move 서비스 | cmd_vel_adapter 추가 |
| woosh_msgs | src/TR-200/woosh_msgs/ | MoveMobile.srv | 유지 |
| woosh_slam_gmapping | src/TR-200/woosh_slam/GMapping/ | 구성됨 | 소폭 수정 |
| woosh_slam_cartographer | src/TR-200/woosh_slam/Cartographer/ | 구성됨 | 소폭 수정 |
| woosh_slam_amcl | src/TR-200/woosh_navigation/AMCL/ | 구성됨 | 소폭 수정 |
| woosh_costmap | src/TR-200/woosh_navigation/Costmap/ | global 구성됨 | local 추가 필요 |

### 2.2 신규 추가 패키지

| 패키지 | 경로 (신규) | 역할 |
|--------|------------|------|
| woosh_localization | src/TR-200/woosh_localization/ | robot_localization EKF 래핑, odom_raw→odom |
| woosh_navigation_mb | src/TR-200/woosh_navigation/Navigation/ | move_base 통합 launch + config |
| woosh_description | src/TR-200/woosh_description/ | URDF/Xacro robot model, footprint |

### 2.3 전체 패키지 역할 표

| 패키지 | 계층 | 주요 역할 | 핵심 파일 |
|--------|------|----------|----------|
| woosh_robot_py | SDK Driver | WebSocket/Protobuf SDK 래핑 | woosh_robot.py, woosh_base.py |
| woosh_msgs | Interface | ROS 커스텀 메시지/서비스 | MoveMobile.srv |
| woosh_sensor_bridge | Bridge | SDK→ROS 변환 (/odom_raw, /scan, TF) | woosh_sensor_bridge.py |
| woosh_bringup | Bringup | /mobile_move 서비스, cmd_vel_adapter, stack launcher | woosh_service_driver.py, cmd_vel_adapter.py |
| woosh_localization | Localization | robot_localization EKF, /odom 발행 | ekf.yaml, localization.launch |
| woosh_slam_gmapping | SLAM | GMapping 맵 생성 | gmapping.launch, gmapping_params.yaml |
| woosh_slam_cartographer | SLAM | Cartographer 맵/위치 추정 | cartographer.launch, *.lua |
| woosh_slam_amcl | Localization | AMCL 위치 추정 | amcl.launch, amcl_params.yaml |
| woosh_costmap | Navigation | costmap 설정 (global/local) | costmap_*.yaml |
| woosh_navigation_mb | Navigation | move_base 통합 | navigation.launch, move_base_params.yaml |
| woosh_description | Description | URDF, footprint, TF static 정의 | woosh_tr200.urdf.xacro |
| main_command | Integration | 갭 탐지 오케스트레이션 | main_system_operation.py |

---

## [3] ROS 인터페이스 명세

### 3.1 토픽 목록

| 토픽 | 방향 | 타입 | 발행 노드 | 구독 노드 | frame_id |
|------|------|------|----------|----------|----------|
| /scan | pub | sensor_msgs/LaserScan | woosh_sensor_bridge | SLAM, move_base | laser |
| /odom_raw | pub | nav_msgs/Odometry | woosh_sensor_bridge | ekf_node | odom |
| /odom | pub | nav_msgs/Odometry | ekf_node (robot_localization) | move_base, SLAM | odom |
| /imu/data_raw | pub | sensor_msgs/Imu | imu_bridge.py (옵션) | ekf_node | imu_link |
| /cmd_vel | sub | geometry_msgs/Twist | move_base | cmd_vel_adapter | - |
| /map | pub | nav_msgs/OccupancyGrid | map_server / SLAM | move_base, AMCL, RViz | map |
| /amcl_pose | pub | geometry_msgs/PoseWithCovarianceStamped | amcl | rviz, main_command | map |
| /initialpose | sub | geometry_msgs/PoseWithCovarianceStamped | rviz | amcl | map |
| /move_base/goal | sub | move_base_msgs/MoveBaseActionGoal | rviz/main | move_base | map |
| /move_base/result | pub | move_base_msgs/MoveBaseActionResult | move_base | main_command | - |
| /mobile_move | service | woosh_msgs/MoveMobile | woosh_service_driver | main_command | - |
| /tf | pub | tf2_msgs/TFMessage | 여러 노드 | 모든 노드 | - |
| /tf_static | pub | tf2_msgs/TFMessage | static_tf_pub | 모든 노드 | - |

### 3.2 frame_id 규칙

| Frame | 의미 | 발행자 |
|-------|------|--------|
| `map` | 전역 좌표계 (SLAM/AMCL 결과) | SLAM/AMCL |
| `odom` | 오도메트리 좌표계 | ekf_node (또는 sensor_bridge fallback) |
| `base_link` | 로봇 기준 좌표계 (중심) | ekf_node TF |
| `base_footprint` | 로봇 바닥 투영 (옵션) | URDF/static |
| `laser` | LiDAR 센서 좌표계 | static_transform_publisher |
| `imu_link` | IMU 센서 좌표계 (옵션) | static_transform_publisher |

### 3.3 TF 트리 (전체)

```
map
├── odom  ← AMCL 또는 SLAM이 발행
│   └── base_link  ← ekf_node 또는 sensor_bridge
│       ├── laser  ← static (x=0, y=0, z=0.25, rpy=0,0,0)
│       └── imu_link  ← static (가정: 로봇 중심에 위치)
```

### 3.4 move_base 인터페이스

| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| move_base (action server) | move_base_msgs/MoveBaseAction | 목표 위치 수신 |
| /move_base/goal | MoveBaseActionGoal | 목표 pose (map frame) |
| /move_base/cancel | actionlib_msgs/GoalID | 취소 |
| /move_base/status | actionlib_msgs/GoalStatusArray | 상태 |
| /move_base/result | MoveBaseActionResult | 결과 |
| /move_base/feedback | MoveBaseActionFeedback | 현재 위치 |
| /move_base/global_costmap/costmap | nav_msgs/OccupancyGrid | 전역 코스트맵 |
| /move_base/local_costmap/costmap | nav_msgs/OccupancyGrid | 지역 코스트맵 |

---

## [4] 단계별 구현 계획

### Phase 1: SDK Bridge 최소 구현

**목표**: SDK로부터 /odom_raw, /scan 발행 확인

**필요 입력**: Woosh 로봇 연결 (WebSocket 169.254.128.2:5480)

**구현 파일**:
- `woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` → `/odom_raw` 발행으로 수정
- `woosh_sensor_bridge/launch/sensor_bridge.launch` → 단독 실행 launch 추가

**검증 방법**:
```bash
rostopic echo /odom_raw
rostopic echo /scan
rosrun tf tf_echo odom base_link
```

**완료 기준**: /odom_raw, /scan 토픽이 10Hz 이상으로 안정적으로 발행됨

---

### Phase 2: cmd_vel → 모바일로봇 구동 연결

**목표**: /cmd_vel 토픽으로 로봇 구동 가능

**구현 파일**:
- `woosh_bringup/scripts/cmd_vel_adapter.py` (신규)
- `woosh_bringup/launch/cmd_vel_adapter.launch` (신규)

**검증 방법**:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.05}" -r 10
# 로봇이 앞으로 이동하는지 확인
```

**완료 기준**: /cmd_vel 토픽 수신 시 로봇이 응답, 토픽 중단 시 1초 내 정지

---

### Phase 3: PoseSpeed.pose → /odom_raw 변환

**목표**: 좌표 원점 설정, covariance 설정, 안정적인 odom_raw 발행

**구현 파일**:
- `woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` 수정
  - 토픽명: `/odom` → `/odom_raw`
  - TF 발행: 조건부 (`publish_tf` 파라미터)
  - 첫 번째 pose를 원점으로 사용

**완료 기준**: /odom_raw의 pose.pose.position이 시작점 기준 상대좌표로 올바르게 발행됨

---

### Phase 4: IMU 연동 또는 Fallback

**가정**: SDK에서 IMU 데이터 제공 안 됨 (현재 확인됨)

**IMU 없을 때 fallback**:
- robot_localization ekf_node를 odom_raw 단독 입력으로 설정
- process_noise 적절히 설정하여 스무딩 효과

**IMU 있을 때 (향후)**:
- `woosh_bringup/scripts/imu_bridge.py` 구현
- SDK에서 IMU raw data 수신 → sensor_msgs/Imu 변환

**완료 기준**: EKF가 /odom_raw를 입력받아 /odom 발행 성공

---

### Phase 5: robot_localization EKF로 /odom 생성

**목표**: EKF를 통해 fused /odom 및 TF(odom→base_link) 발행

**구현 파일**:
- `woosh_localization/config/ekf_no_imu.yaml` (신규)
- `woosh_localization/launch/ekf_localization.launch` (신규)
- `woosh_localization/package.xml` (신규)

**패키지 의존**: robot_localization

**검증 방법**:
```bash
rostopic echo /odom
rosrun tf tf_echo odom base_link
# EKF 없을 때와 비교하여 trajectory 스무딩 확인
```

**완료 기준**: /odom 토픽 발행 + TF(odom→base_link) 중복 없이 단일 발행

---

### Phase 6: LiDAR → /scan 연결 확인

**목표**: /scan 데이터 품질 검증, frame_id/타임스탬프 정확성 확인

**구현 파일**:
- `woosh_bringup/launch/static_tf.launch` → laser static TF

**검증 방법**:
```bash
rviz → LaserScan 표시, base_link 기준으로 올바른 방향인지 확인
rostopic hz /scan  # 10Hz 이상 확인
```

**완료 기준**: RViz에서 /scan이 로봇 주변에 올바르게 표시됨

---

### Phase 7: TF 정리

**목표**: TF 트리 완성, 중복 TF 제거

**확인 사항**:
- sensor_bridge의 TF 발행 비활성화 (`publish_tf: false`)
- ekf_node가 odom→base_link 발행
- SLAM/AMCL이 map→odom 발행
- static: base_link→laser

**검증 방법**:
```bash
rosrun tf view_frames
# frames.pdf 확인: map→odom→base_link→laser 체인
rosrun rqt_tf_tree rqt_tf_tree
```

**완료 기준**: TF 트리에 중복 없음, 완전한 map→laser 체인 성립

---

### Phase 8: RViz 기본 검증

**목표**: 전체 센서 데이터 시각화 확인

**구현 파일**:
- `woosh_bringup/rviz/sensor_debug.rviz`
- `woosh_bringup/launch/sensor_debug.launch`

**검증 방법**:
- RViz에서 /scan, /odom, TF 시각화
- 로봇 이동 시 odom 궤적 확인
- LiDAR scan과 실제 환경 일치 확인

**완료 기준**: RViz에서 로봇 이동과 센서 데이터가 정합성 있게 표시됨

---

### Phase 9: GMapping/Cartographer로 map 생성

**목표**: SLAM으로 실내 지도 생성

**현재 상태**: 이미 launch 파일 존재 (수정 필요)

**수정 사항**:
- `woosh_slam_gmapping/launch/gmapping.launch` → /odom 구독 확인
- `woosh_slam_cartographer/launch/cartographer.launch` → 동일

**검증 방법**:
```bash
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2
# RViz에서 /map 증분 업데이트 확인
# 로봇 수동 조작으로 환경 탐색
```

**완료 기준**: 맵 완성 후 map_saver로 저장 성공

---

### Phase 10: map_server + AMCL 구성

**목표**: 저장된 맵 기반 위치 추정

**현재 상태**: amcl.launch 이미 존재

**구현/수정 파일**:
- `woosh_slam_amcl/config/amcl_params.yaml` 튜닝
- `woosh_slam_amcl/launch/amcl.launch` → /odom 구독 확인

**검증 방법**:
```bash
roslaunch woosh_slam_amcl amcl.launch map_file:=/path/to/map.yaml robot_ip:=169.254.128.2
# RViz에서 amcl particle cloud 확인
# /amcl_pose 토픽 안정성 확인
```

**완료 기준**: AMCL particle이 수렴, /amcl_pose 발행, TF(map→odom) 안정

---

### Phase 11: move_base 연동

**목표**: move_base를 통한 자율 이동 명령 처리

**신규 구현 파일**:
- `woosh_navigation_mb/launch/navigation.launch`
- `woosh_navigation_mb/config/move_base_params.yaml`
- `woosh_navigation_mb/config/global_planner_params.yaml`
- `woosh_navigation_mb/config/local_planner_params.yaml`
- `woosh_navigation_mb/config/global_costmap_params.yaml`
- `woosh_navigation_mb/config/local_costmap_params.yaml`

**패키지 의존**: move_base, navfn, dwa_local_planner 또는 teb_local_planner

**검증 방법**:
```bash
# RViz에서 2D Nav Goal 클릭 후 로봇 이동 확인
rostopic echo /move_base/result
```

**완료 기준**: 목표점 전송 시 로봇이 장애물 회피하며 목표에 도달

---

### Phase 12: 실제 Navigation 테스트 및 튜닝

**목표**: 반복 navigation 안정성 확인 + costmap 최적화

**테스트 시나리오**:
1. 시작점 → 목표점 1회 navigation
2. 여러 waypoint 순차 navigation
3. 동적 장애물 대응 확인

**튜닝 항목**:
- inflation_radius, obstacle layer 최적화
- recovery_behaviors 설정
- TR-200 실제 footprint 반영

**완료 기준**: 3회 연속 navigation 성공, 장애물 감지 시 우회

---

## [5] 파일/폴더 스캐폴딩

### 5.1 woosh_sensor_bridge (수정)

```
woosh_sensor_bridge/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   └── woosh_sensor_bridge.py        # /odom → /odom_raw 변경, publish_tf 파라미터 추가
└── launch/
    └── sensor_bridge.launch          # 단독 실행용 (신규)
```

### 5.2 woosh_bringup (수정/추가)

```
woosh_bringup/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   ├── woosh_service_driver.py       # 기존 유지
│   ├── cmd_vel_adapter.py            # 신규: /cmd_vel → SDK twist_req
│   └── woosh_rviz_debug.py           # 기존 유지
├── launch/
│   ├── woosh_rviz_debug.launch       # 기존 유지
│   ├── sensor_bridge.launch          # 신규: sensor bridge만 기동
│   ├── cmd_vel_adapter.launch        # 신규: cmd_vel adapter
│   ├── bringup.launch                # 수정: sensor_bridge + cmd_vel_adapter + static_tf
│   └── static_tf.launch              # 신규: base_link→laser static TF
├── rviz/
│   ├── sensor_debug.rviz             # 신규: 센서 디버그 뷰
│   └── navigation.rviz               # 신규: 네비게이션 뷰
└── config/
    └── robot_params.yaml             # 신규: ip, port, 속도 제한 등
```

### 5.3 woosh_localization (신규 패키지)

```
woosh_localization/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── ekf_no_imu.yaml               # IMU 없을 때 EKF 설정
│   └── ekf_with_imu.yaml             # IMU 있을 때 EKF 설정 (향후)
└── launch/
    ├── ekf_localization.launch       # robot_localization ekf_node 기동
    └── localization.launch           # ekf + static_tf 통합
```

### 5.4 woosh_description (신규 패키지)

```
woosh_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── woosh_tr200.urdf.xacro        # 로봇 URDF (footprint, laser 위치)
├── meshes/                            # 3D 모델 (선택)
└── launch/
    └── robot_description.launch      # robot_state_publisher 기동
```

### 5.5 woosh_slam_gmapping (수정)

```
woosh_slam_gmapping/
├── config/
│   └── gmapping_params.yaml          # 기존 유지/튜닝
└── launch/
    ├── gmapping.launch               # 수정: /odom 구독 확인
    └── save_map.launch               # 기존 유지
```

### 5.6 woosh_slam_amcl (수정)

```
woosh_slam_amcl/
├── config/
│   └── amcl_params.yaml              # 수정: odom_model_type, min/max particles
└── launch/
    ├── amcl.launch                   # 수정: /odom 구독 확인
    └── localization_only.launch      # 신규: map_server 없이 amcl만
```

### 5.7 woosh_navigation_mb (신규 패키지)

```
woosh_navigation_mb/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── move_base_params.yaml         # move_base 공통 설정
│   ├── costmap_common_params.yaml    # global/local 공통 costmap
│   ├── global_costmap_params.yaml    # global costmap
│   ├── local_costmap_params.yaml     # local costmap
│   ├── global_planner_params.yaml    # navfn 또는 global_planner
│   └── local_planner_params.yaml     # dwa_local_planner 또는 teb
└── launch/
    ├── navigation.launch             # move_base + amcl + map_server
    └── move_base_only.launch         # map/amcl 없이 move_base만
```

---

## [6] 핵심 Python 노드 책임과 의사코드

### 6.1 woosh_sensor_bridge.py (수정)

**역할**: SDK PoseSpeed, ScannerData → ROS /odom_raw, /scan, TF

**Subscribe**: 없음 (SDK 직접 구독)
**Publish**: /odom_raw (nav_msgs/Odometry), /scan (LaserScan), TF(odom→base_link) 조건부

```python
class WooshSensorBridgeNode:
    def __init__(self):
        rospy.init_node('woosh_sensor_bridge')

        # 파라미터
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        # EKF 사용 시 False로 설정

        # Publisher - 토픽명 변경: /odom → /odom_raw
        self.odom_pub = rospy.Publisher('/odom_raw', Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # SDK 초기화
        self.robot_adapter = WooshRobotAdapter(ip=self.robot_ip)

        # 원점 설정용
        self.origin_pose = None
        self.origin_initialized = False

    def pose_callback(self, sdk_pose_speed):
        """
        SDK PoseSpeed → nav_msgs/Odometry (/odom_raw)

        주의: SDK pose는 로봇 내부 좌표계일 수 있음
        → 첫 수신값을 원점으로 상대화
        """
        if not self.origin_initialized:
            self.origin_pose = sdk_pose_speed.pose
            self.origin_initialized = True

        # 상대 pose 계산
        rel_x, rel_y, rel_theta = self._compute_relative_pose(
            sdk_pose_speed.pose, self.origin_pose
        )

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = rel_x
        odom_msg.pose.pose.position.y = rel_y
        odom_msg.pose.pose.orientation = euler_to_quaternion(0, 0, rel_theta)

        # covariance: 대각 성분만 설정 (empirical values)
        odom_msg.pose.covariance[0] = 0.05   # x
        odom_msg.pose.covariance[7] = 0.05   # y
        odom_msg.pose.covariance[35] = 0.1   # yaw

        odom_msg.twist.twist.linear.x = sdk_pose_speed.twist.linear
        odom_msg.twist.twist.angular.z = sdk_pose_speed.twist.angular
        odom_msg.twist.covariance[0] = 0.001
        odom_msg.twist.covariance[35] = 0.005

        self.odom_pub.publish(odom_msg)

        # TF 발행 (EKF 없을 때만)
        if self.publish_tf:
            self._publish_tf(rel_x, rel_y, rel_theta)

    # 예외처리: SDK 연결 끊김 시 publish 중단 + 재연결 시도
    # 예외처리: pose가 (0,0,0)으로 고정 시 경고 로그
```

### 6.2 cmd_vel_adapter.py (신규)

**역할**: /cmd_vel 토픽 → SDK twist_req() 전달

**Subscribe**: /cmd_vel (geometry_msgs/Twist)
**Publish**: 없음 (SDK 직접 제어)

```python
class CmdVelAdapter:
    def __init__(self):
        rospy.init_node('cmd_vel_adapter')

        # 속도 제한 파라미터
        self.max_linear = rospy.get_param('~max_linear_vel', 0.12)  # m/s
        self.max_angular = rospy.get_param('~max_angular_vel', 0.5)  # rad/s
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 1.0)  # seconds

        # SDK adapter
        self.robot_adapter = WooshRobotAdapter(
            ip=rospy.get_param('~robot_ip', '169.254.128.2')
        )

        # Watchdog: cmd_vel 타임아웃 시 정지
        self.last_cmd_time = rospy.Time(0)
        self.watchdog_timer = rospy.Timer(
            rospy.Duration(0.1), self._watchdog_callback
        )

        # Subscriber
        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)

    def _cmd_vel_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()

        # 속도 제한 클리핑
        vx = np.clip(msg.linear.x, -self.max_linear, self.max_linear)
        wz = np.clip(msg.angular.z, -self.max_angular, self.max_angular)

        # SDK 명령 전송 (비동기 → asyncio event loop 활용)
        self._send_twist(vx, wz)

    def _watchdog_callback(self, event):
        """cmd_vel 토픽 타임아웃 시 자동 정지"""
        dt = (rospy.Time.now() - self.last_cmd_time).to_sec()
        if dt > self.cmd_timeout:
            self._send_twist(0.0, 0.0)  # 정지 명령

    def _send_twist(self, linear, angular):
        """SDK adapter를 통해 twist 명령 전송"""
        try:
            self.robot_adapter.send_twist(linear, angular)
        except Exception as e:
            rospy.logwarn(f"[CmdVelAdapter] twist 전송 실패: {e}")

    # 예외처리: SDK 연결 끊김 → rospy.logerr + 노드 재시작 권고
    # 예외처리: 비선형 로봇(mecanum) 대응 → vy 파라미터 확장 가능
```

### 6.3 scan_callback (참고 — 현재 woosh_sensor_bridge에 통합됨)

```python
def scan_callback(self, sdk_scan_data):
    """
    SDK ScannerData → sensor_msgs/LaserScan

    주의:
    - angle_min/max가 로봇 기준인지 LiDAR 기준인지 확인 필요
    - ranges 배열 방향(CW/CCW) 확인 필요
    """
    scan_msg = LaserScan()
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = 'laser'

    scan_msg.angle_min = sdk_scan_data.angle_min
    scan_msg.angle_max = sdk_scan_data.angle_max
    scan_msg.angle_increment = sdk_scan_data.angle_increment
    scan_msg.time_increment = sdk_scan_data.time_increment
    scan_msg.scan_time = sdk_scan_data.scan_time
    scan_msg.range_min = sdk_scan_data.range_min
    scan_msg.range_max = sdk_scan_data.range_max
    scan_msg.ranges = list(sdk_scan_data.ranges)

    # 예외처리: ranges가 비어있을 때 skip
    # 예외처리: range_min/max 이상값 필터링

    self.scan_pub.publish(scan_msg)
```

### 6.4 imu_bridge.py (옵션, 향후)

```python
class ImuBridge:
    def publish_imu(self, sdk_imu_data):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_link'

        # 가정: sdk_imu_data.linear_acceleration.{x,y,z}
        # 가정: sdk_imu_data.angular_velocity.{x,y,z}
        # 가정: sdk_imu_data.orientation (quaternion)

        # covariance 미제공 시 -1 설정 (unknown)
        imu_msg.orientation_covariance[0] = -1

        self.imu_pub.publish(imu_msg)
```

---

## [7] EKF / Localization 설계

### 7.1 IMU 없을 때 EKF 설정

**전략**: odom_raw 단독 입력 EKF (필터 + 스무딩 효과)

```yaml
# ekf_no_imu.yaml
frequency: 50  # Hz

two_d_mode: true  # 2D 평면 이동

odom0: /odom_raw
odom0_config: [true,  true,  false,   # x, y, z
               false, false, true,    # roll, pitch, yaw
               true,  true,  false,   # vx, vy, vz
               false, false, true,    # vroll, vpitch, vyaw
               false, false, false]   # ax, ay, az

odom0_differential: false
# PoseSpeed.pose는 절대 좌표이므로 differential=false

odom0_relative: false

process_noise_covariance: [0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
                           0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015]

world_frame: odom
map_frame: map
odom_frame: odom
base_link_frame: base_link

transform_time_offset: 0.0
transform_timeout: 0.0

publish_tf: true
print_diagnostics: true
```

### 7.2 IMU 있을 때 EKF 설정 (향후)

```yaml
# ekf_with_imu.yaml — odom0 유지 + imu0 추가
imu0: /imu/data_raw
imu0_config: [false, false, false,
              false, false, true,     # yaw만 사용
              false, false, false,
              false, false, true,     # yaw rate
              false, false, false]

imu0_differential: false
imu0_remove_gravitational_acceleration: true
```

### 7.3 Yaw 안정화 주의사항

1. **differential=false** 필수: PoseSpeed.pose는 절대 좌표 (오도미터 리셋 없음)
2. **odom drift**: SDK 내부 휠 오도미터 기반 → 장거리 주행 시 오차 누적
3. **TF 충돌**: sensor_bridge와 ekf_node 모두 TF 발행 시 오류 → `publish_tf` 파라미터로 제어
4. **covariance 조정**: 실제 로봇 환경에서 empirical tuning 필요

---

## [8] SLAM 전략 비교

### 8.1 GMapping vs Cartographer

| 항목 | GMapping | Cartographer |
|------|----------|--------------|
| 방식 | Particle filter SLAM | Graph-based optimization SLAM |
| odom 의존도 | 높음 | 낮음 (loop closure로 보정) |
| IMU 지원 | 없음 | 있음 (선택) |
| 맵 품질 | 작은 공간에서 충분 | loop closure로 더 정확 |
| CPU | 낮음 | 높음 |
| 설정 복잡도 | 낮음 | 높음 (.lua 파일) |
| localization 전환 | AMCL 별도 필요 | pure localization 내장 |
| 이미 설정됨 | O | O |

### 8.2 추천 전략

**초기 개발 단계**: **GMapping 우선**

이유:
- 설정이 단순하여 sensor bridge 검증에 집중 가능
- odom 품질에 덜 민감 (particle filter 특성)
- IMU 없이도 작동
- 작은 실내 공간에서 충분한 맵 품질
- `woosh_slam_gmapping` 패키지 이미 완성

**중/장기 목표**: Cartographer

이유:
- loop closure로 대형 공간 맵 품질 우수
- cartographer pure localization으로 별도 AMCL 불필요
- `woosh_slam_cartographer` 패키지 이미 완성

**권장 순서**:
1. GMapping으로 첫 맵 생성 + AMCL 검증
2. Cartographer로 맵 품질 비교
3. 요구사항에 따라 선택

---

## [9] Navigation 구조 설계

### 9.1 전체 Navigation Stack

```
map_server ─────────── /map ──────────────→ move_base
amcl ────────────────── TF(map→odom) ────→ move_base
/odom ───────────────────────────────────→ move_base
/scan ───────────────────────────────────→ move_base (local costmap obstacle)
move_base ──────────── /cmd_vel ─────────→ cmd_vel_adapter → 로봇
```

### 9.2 move_base 공통 설정

```yaml
# move_base_params.yaml
base_global_planner: "navfn/NavfnROS"
base_local_planner: "dwa_local_planner/DWAPlannerROS"

controller_frequency: 5.0  # Hz (Woosh 로봇 응답 속도 고려)
planner_patience: 5.0
controller_patience: 15.0
conservative_reset_dist: 3.0

recovery_behaviors:
  - name: 'conservative_reset'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
  - name: 'rotate_recovery'
    type: 'rotate_recovery/RotateRecovery'
  - name: 'aggressive_reset'
    type: 'clear_costmap_recovery/ClearCostmapRecovery'
```

### 9.3 Costmap 설정

```yaml
# costmap_common_params.yaml
robot_radius: 0.25  # 가정: TR-200 반지름 25cm (측정 필요)
# 직사각형 footprint 사용 시:
# footprint: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]

obstacle_range: 3.0
raytrace_range: 3.5

inflation_radius: 0.35
cost_scaling_factor: 5.0

observation_sources: laser_scan_sensor
laser_scan_sensor: {
  sensor_frame: laser,
  data_type: LaserScan,
  topic: /scan,
  marking: true,
  clearing: true
}
```

```yaml
# global_costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 0.5
  static_map: true
  transform_tolerance: 0.5
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

```yaml
# local_costmap_params.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 3.0
  height: 3.0
  resolution: 0.05
  transform_tolerance: 0.5
  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

### 9.4 구동 방식별 변경 사항

| 항목 | Differential Drive | Mecanum/Omnidirectional |
|------|-------------------|------------------------|
| cmd_vel.linear.y | 사용 안 함 | 사용 |
| local planner | DWA (추천) | TEB 추천 |
| odom_model_type (AMCL) | diff | omni |
| cmd_vel_adapter.py | linear.x + angular.z만 | linear.x/y + angular.z |
| footprint | 대칭 | 실제 형상 |

> **가정**: TR-200은 Differential Drive (2WD + 캐스터)
> → 확인 필요: SDK Twist에서 yaw가 angular.z인지 확인

---

## [10] Launch 전략

### 10.1 launch 파일 계층

```
bringup.launch
├── sensor_bridge.launch    (woosh_sensor_bridge)
├── static_tf.launch        (base_link→laser)
├── cmd_vel_adapter.launch  (cmd_vel → SDK)
└── robot_description.launch (urdf→/robot_description)

ekf_localization.launch
├── ekf_node (robot_localization)
└── (bringup.launch 포함)

slam_gmapping.launch
├── bringup.launch
├── ekf_localization.launch
├── gmapping_node
└── rviz (선택)

slam_cartographer.launch
├── bringup.launch
├── ekf_localization.launch
├── cartographer_node
├── cartographer_occupancy_grid_node
└── rviz (선택)

save_map.launch
└── map_saver (-f 맵파일명)

localization_amcl.launch
├── bringup.launch
├── ekf_localization.launch
├── map_server (맵 파일)
├── amcl_node
└── rviz (선택)

navigation.launch
├── localization_amcl.launch
└── move_base_node
```

### 10.2 단계별 실행 명령

```bash
# 1. bringup (센서만)
roslaunch woosh_bringup bringup.launch robot_ip:=169.254.128.2

# 2. 센서 확인
rostopic echo /odom_raw
rostopic echo /scan
rosrun tf view_frames

# 3. RViz 디버그
roslaunch woosh_bringup sensor_debug.launch robot_ip:=169.254.128.2

# 4. SLAM (GMapping)
roslaunch woosh_slam_gmapping slam_gmapping.launch robot_ip:=169.254.128.2

# 5. 맵 저장
roslaunch woosh_slam_gmapping save_map.launch map_name:=my_map

# 6. 위치추정 (AMCL)
roslaunch woosh_slam_amcl localization_amcl.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/my_map.yaml

# 7. Navigation
roslaunch woosh_navigation_mb navigation.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/my_map.yaml
```

---

## [11] 리스크와 우선순위

### 11.1 주요 기술 리스크

| 리스크 | 심각도 | 발생 가능성 | 대응 방안 |
|--------|--------|------------|----------|
| SDK PoseSpeed.pose 좌표계 불일치 | 높음 | 중간 | 원점 상대화, 실험적 검증 |
| /odom_raw → /odom 변경으로 기존 패키지 깨짐 | 중간 | 높음 | woosh_service_driver odom 구독 경로 수정 |
| TF 중복 발행 (sensor_bridge + ekf_node) | 높음 | 높음 | publish_tf 파라미터 제어 |
| IMU 없음으로 EKF 효과 제한 | 중간 | 높음 | odom_raw 단독 EKF로 fallback |
| cmd_vel watchdog 미구현 시 로봇 폭주 | 매우 높음 | 낮음 | watchdog timer 필수 구현 |
| SDK API 변경 시 adapter 깨짐 | 중간 | 낮음 | adapter 인터페이스 계층 분리 |
| Docker 내 asyncio + rospy 충돌 | 중간 | 중간 | separate thread/event loop |

### 11.2 Navigation 전에 반드시 해결해야 할 항목

1. /odom_raw 안정적 발행 (10Hz 이상, drift 허용 범위 내)
2. /scan 정확한 frame_id 및 방향 확인
3. TF 트리 완성 (odom→base_link→laser, 중복 없음)
4. cmd_vel_adapter watchdog 동작 확인
5. robot_localization EKF /odom 발행

### 11.3 지금 당장 vs 나중에

**지금 당장 (MVP)**:
- woosh_sensor_bridge /odom_raw 변경
- cmd_vel_adapter.py 구현
- static_tf base_link→laser 확인
- ekf_no_imu.yaml + ekf_localization.launch
- GMapping 동작 확인
- AMCL 동작 확인

**나중에 고도화**:
- IMU 연동 (SDK 확인 후)
- Cartographer pure localization
- TEB local planner (좁은 공간)
- woosh_description URDF 완성
- multi-robot 통합 (Doosan + TR-200 동시)
- dynamic reconfigure for costmap

### 11.4 SDK 기반 Pose 한계

- 로봇 내부 추정값 → 슬립, 충격 시 오차 누적
- 장거리 이동 후 오차 수 cm ~ 수십 cm 발생 가능
- SLAM loop closure 없이는 drift 보정 불가
- **대응**: AMCL 사용 시 map-based correction이 drift를 보정

### 11.5 로봇 내부 ROS Master 전환 시점

아래 상황에서만 전환 고려:
- SDK에서 제공 불가한 고주파 센서 데이터 필요 (예: 20Hz+ IMU)
- 로봇 내부 ROS 토픽이 직접 필요한 서드파티 패키지
- 현재 SDK로는 제어 불가한 기능 필요
- **현재는 전환 불필요**

---

## [12] 요약

### A. 추천 최종 구조

```
Docker ROS Noetic
├── woosh_sensor_bridge     → /odom_raw, /scan, TF(옵션)
├── cmd_vel_adapter         → /cmd_vel → SDK
├── robot_localization      → /odom, TF(odom→base_link)
├── static_tf_publisher     → TF(base_link→laser)
├── map_server              → /map (navigation 시)
├── amcl                    → TF(map→odom) (navigation 시)
└── move_base               → /cmd_vel (목표 기반 자율 이동)
```

### B. 우선 구현 1순위 파일

1. `woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` (수정: /odom_raw)
2. `woosh_bringup/scripts/cmd_vel_adapter.py` (신규)
3. `woosh_localization/config/ekf_no_imu.yaml` (신규)
4. `woosh_localization/launch/ekf_localization.launch` (신규)
5. `woosh_bringup/launch/bringup.launch` (수정: sensor_bridge + cmd_vel_adapter + static_tf)
6. `woosh_bringup/launch/static_tf.launch` (신규)

### C. MVP 구현 순서

1. sensor_bridge /odom_raw 변경 + 검증
2. cmd_vel_adapter 구현 + watchdog 확인
3. static_tf base_link→laser 확인
4. ekf_no_imu.yaml 설정 + /odom 발행 확인
5. TF 트리 완성 확인
6. GMapping SLAM 테스트
7. map 저장
8. AMCL localization 테스트

### D. Navigation 완성까지 확장 순서

1. woosh_navigation_mb 패키지 생성
2. move_base_params.yaml, costmap 설정
3. navigation.launch 구성
4. RViz 2D Nav Goal 테스트
5. footprint 실측 후 URDF 업데이트
6. costmap inflation 튜닝
7. recovery behavior 테스트
8. main_command 통합 (navigation + gap detection)
