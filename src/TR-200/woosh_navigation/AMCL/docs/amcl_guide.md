# woosh_slam_amcl — AMCL 로컬리제이션 가이드

Woosh TR-200 모바일 로봇을 위한 AMCL(Adaptive Monte Carlo Localization) 패키지입니다.
사전에 제작된 맵 위에서 파티클 필터를 이용해 로봇의 위치를 실시간으로 추정합니다.

---

## 목차

1. [개요](#개요)
2. [패키지 구조](#패키지-구조)
3. [TF 트리 설계](#tf-트리-설계)
4. [노드 구성](#노드-구성)
5. [빠른 시작](#빠른-시작)
6. [파라미터 레퍼런스](#파라미터-레퍼런스)
7. [woosh_service_driver 통합 실행](#woosh_service_driver-통합-실행)
8. [RViz 시각화](#rviz-시각화)
9. [튜닝 가이드](#튜닝-가이드)
10. [트러블슈팅](#트러블슈팅)
11. [설계 결정 배경](#설계-결정-배경)

---

## 개요

AMCL은 **알려진 맵 위에서** 로봇이 현재 어디에 있는지를 추정하는 로컬리제이션 알고리즘입니다.
지도 생성(SLAM)이 아닌, **이미 만들어진 맵을 활용한 위치 추정**에 사용합니다.

```
[맵 파일 (.pgm + .yaml)]
        │
        ▼
   map_server ──────────────────────► /map 토픽

[Woosh TR-200 SDK]
        │
        ▼
woosh_sensor_bridge ──► /scan  (레이저 스캔)
                    ──► /odom  (합성 오도메트리)
                    ──► TF: odom → base_link

        │ /scan + /map + /odom + TF
        ▼
      amcl ──► /amcl_pose       (추정 위치, PoseWithCovarianceStamped)
           ──► /particlecloud   (파티클 분포, PoseArray)
           ──► TF: map → odom   (오도메트리 드리프트 보정)
```

### 전제 조건

- Woosh TR-200과 WebSocket 연결 가능 상태 (`169.254.128.2:5480`)
- 맵 파일 (`.pgm` + `.yaml`) 존재 — 없으면 [맵 내보내기](#1-맵-내보내기) 먼저 수행
- ROS1 Noetic 환경 (Docker 컨테이너 내부)

---

## 패키지 구조

```
woosh_navigation/AMCL/                     ← ROS 패키지: woosh_slam_amcl
├── package.xml
├── CMakeLists.txt
├── scripts/
│   └── export_map.py                # Woosh 맵 → .pgm/.yaml 변환 유틸리티
│   # woosh_sensor_bridge.py는 woosh_sensor_bridge 패키지로 이동
├── launch/
│   └── amcl.launch                  # 전체 AMCL 스택 런치 파일
├── config/
│   └── amcl_params.yaml             # AMCL 알고리즘 파라미터 (TR-200 튜닝)
├── rviz/
│   └── amcl_debug.rviz              # AMCL 시각화 RViz 설정
└── docs/
    └── amcl_guide.md                # 이 문서
```

---

## TF 트리 설계

```
map (고정 좌표계)
 └── odom          ← amcl이 발행 (파티클 필터 보정값)
      └── base_link ← woosh_sensor_bridge.py 발행 (twist 적분)
           └── laser ← static_transform_publisher (고정 오프셋)
```

| 변환 | 발행자 | 내용 |
|------|--------|------|
| `map → odom` | `amcl` 노드 | 오도메트리 드리프트를 레이저/맵 매칭으로 보정 |
| `odom → base_link` | `woosh_sensor_bridge.py` | PoseSpeed.twist 시간 적분 (합성 오도메트리) |
| `base_link → laser` | `static_transform_publisher` | 로봇 상의 LiDAR 고정 위치 |

### 합성 오도메트리 계산

Woosh SDK는 바퀴 엔코더를 직접 노출하지 않으므로, `PoseSpeed.twist`의 `linear` / `angular`를 시간 적분하여 오도메트리를 합성합니다.

```
x(t)     += linear · cos(θ) · dt
y(t)     += linear · sin(θ) · dt
θ(t)     += angular · dt
```

합성 오도메트리는 드리프트가 누적되지만, AMCL의 파티클 필터가 레이저 스캔과 맵을 비교하여 이를 지속적으로 보정합니다.

---

## 노드 구성

### 1. `woosh_sensor_bridge` (`woosh_sensor_bridge.py`)

Woosh SDK와 ROS 토픽 사이의 브릿지 역할을 합니다.

| 항목 | 값 |
|------|----|
| SDK identity | `amcl_bridge` |
| 발행 토픽 | `/scan` (LaserScan), `/odom` (Odometry) |
| 발행 TF | `odom → base_link` |
| 기본 주기 | 10 Hz |
| 폴링 간격 | 0.1 s (SDK 구독 + 요청 혼합) |

주요 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `~robot_ip` | `169.254.128.2` | 로봇 IP |
| `~robot_port` | `5480` | 로봇 포트 |
| `~publish_hz` | `10.0` | 발행 주기 (Hz) |
| `~odom_frame` | `odom` | 오도메트리 프레임 이름 |
| `~base_frame` | `base_link` | 로봇 베이스 프레임 이름 |
| `~laser_frame` | `laser` | 레이저 프레임 이름 |

### 2. `map_server`

저장된 맵 파일을 `/map` 토픽으로 서빙합니다.

| 항목 | 값 |
|------|----|
| 발행 토픽 | `/map` (OccupancyGrid) |
| 맵 파일 | `amcl.launch`의 `map_file` 인자로 지정 |

### 3. `amcl`

파티클 필터 기반 로컬리제이션을 수행합니다.

| 항목 | 값 |
|------|----|
| 구독 토픽 | `/scan`, `/map`, `/initialpose` |
| 발행 토픽 | `/amcl_pose`, `/particlecloud` |
| 발행 TF | `map → odom` |
| 파라미터 파일 | `config/amcl_params.yaml` |

### 4. `static_transform_publisher`

`base_link → laser` 정적 변환을 발행합니다.
LiDAR가 로봇 중심에서 `z=0.25m` 높이에 장착된 것으로 기본 설정되어 있습니다.

---

## 빠른 시작

모든 명령은 **Docker 컨테이너 내부**(`/root/catkin_ws`)에서 실행합니다.

### 0. 빌드 (최초 1회)

```bash
catkin build woosh_slam_amcl
source devel/setup.bash
```

### 1. 맵 내보내기

로봇에 저장된 맵을 ROS `map_server` 포맷으로 저장합니다.

```bash
rosrun woosh_slam_amcl export_map.py \
  _robot_ip:=169.254.128.2 \
  _output_dir:=/root/catkin_ws/src/TR-200/woosh_slam/maps \
  _map_name:=woosh_map
```

성공 시 `/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.pgm` 과 `woosh_map.yaml`이 생성됩니다.

추가 옵션:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `_robot_ip` | `169.254.128.2` | 로봇 IP |
| `_output_dir` | `/root/catkin_ws/src/TR-200/woosh_slam/maps` | 저장 경로 |
| `_map_name` | `woosh_map` | 파일 이름 (확장자 제외) |
| `_scene_name` | (로봇의 현재 scene) | 특정 scene 지정 |
| `_map_name_filter` | (첫 번째 맵) | scene 내 특정 맵 이름 지정 |

### 2. AMCL 스택 단독 실행

```bash
roslaunch woosh_slam_amcl amcl.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml
```

RViz 없이 실행하려면:

```bash
roslaunch woosh_slam_amcl amcl.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml \
  launch_rviz:=false
```

### 3. 초기 위치 설정

RViz가 실행된 후 로봇의 대략적인 위치를 수동으로 지정해야 합니다.

1. RViz 상단 툴바에서 **2D Pose Estimate** 클릭
2. 맵 위에서 로봇의 현재 위치를 클릭하고 드래그하여 방향 설정
3. `/particlecloud` (파란 화살표들)가 로봇 주변에 수렴하면 로컬리제이션 성공

> 로봇을 조금 이동시키거나 회전시키면 파티클이 빠르게 수렴합니다.

### 4. 로컬리제이션 확인

```bash
# AMCL 추정 위치 확인
rostopic echo /amcl_pose

# 파티클 수렴 상태 확인
rostopic hz /particlecloud   # 약 10 Hz

# TF 트리 확인
rosrun tf tf_echo map base_link
```

---

## 파라미터 레퍼런스

### amcl.launch 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_ip` | `169.254.128.2` | Woosh 로봇 IP |
| `robot_port` | `5480` | Woosh 로봇 포트 |
| `map_file` | `/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml` | 맵 파일 경로 |
| `map_frame` | `map` | 맵 프레임 이름 |
| `odom_frame` | `odom` | 오도메트리 프레임 이름 |
| `base_frame` | `base_link` | 로봇 베이스 프레임 이름 |
| `laser_frame` | `laser` | 레이저 프레임 이름 |
| `laser_offset_x` | `0.0` | LiDAR x 오프셋 (m) |
| `laser_offset_y` | `0.0` | LiDAR y 오프셋 (m) |
| `laser_offset_z` | `0.25` | LiDAR z 오프셋 (m) |
| `laser_offset_yaw` | `0.0` | LiDAR yaw 오프셋 (rad) |
| `publish_hz` | `10.0` | 센서 브릿지 발행 주기 (Hz) |
| `launch_rviz` | `true` | RViz 자동 실행 여부 |

### amcl_params.yaml 주요 파라미터

#### 파티클 필터

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `min_particles` | `500` | 최소 파티클 수 |
| `max_particles` | `3000` | 최대 파티클 수 (NUC 성능 기준) |
| `update_min_d` | `0.05` | 업데이트 트리거 이동 거리 (m) |
| `update_min_a` | `0.1745` | 업데이트 트리거 회전 각도 (rad, ≈10°) |
| `resample_interval` | `2` | 리샘플링 간격 (업데이트 횟수) |

#### 레이저 모델

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `laser_model_type` | `likelihood_field` | 실내 환경에 강건한 모델 |
| `laser_max_beams` | `60` | 서브샘플링 빔 수 (CPU 부하 제어) |
| `laser_z_hit` | `0.95` | hit 모델 가중치 |
| `laser_z_rand` | `0.05` | 랜덤 노이즈 가중치 |
| `laser_sigma_hit` | `0.2` | hit 가우시안 표준편차 (m) |
| `laser_likelihood_max_dist` | `2.0` | likelihood field 최대 거리 (m) |

#### 오도메트리 모델 (차동 구동)

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `odom_model_type` | `diff` | 차동 구동 모델 |
| `odom_alpha1` | `0.3` | 회전으로 인한 회전 노이즈 |
| `odom_alpha2` | `0.3` | 이동으로 인한 회전 노이즈 |
| `odom_alpha3` | `0.2` | 이동으로 인한 이동 노이즈 |
| `odom_alpha4` | `0.2` | 회전으로 인한 이동 노이즈 |

> alpha 값이 클수록 오도메트리 노이즈가 크다고 가정합니다. 합성 오도메트리이므로 보수적으로 설정했습니다.

#### 기타

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `transform_tolerance` | `0.5` | TF 유효 시간 허용 오차 (s) — WebSocket 지연 고려 |
| `gui_publish_rate` | `10.0` | 파티클 시각화 발행 주기 (Hz) |
| `recovery_alpha_slow` | `0.001` | 느린 평균 가중치 감쇠율 |
| `recovery_alpha_fast` | `0.1` | 빠른 평균 가중치 감쇠율 (복구 감지) |

---

## woosh_service_driver 통합 실행

`woosh_service_driver.py`에서 `amcl` 플래그를 사용하면 `/mobile_move` 서비스와 AMCL을 동시에 실행할 수 있습니다.

```bash
# 기본 맵 파일 사용
rosrun woosh_bringup woosh_service_driver.py amcl

# 맵 파일 경로 지정
rosrun woosh_bringup woosh_service_driver.py amcl map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/my_map.yaml
```

`amcl` 플래그를 사용하면 자동으로 `rviz_on`이 활성화되며, RViz는 `amcl_debug.rviz` 설정으로 실행됩니다.

### 프로세스 구성

```
woosh_service_driver.py (메인 프로세스)
  ├── SmoothTwistController (asyncio 스레드)
  │     └── SDK 연결: robot_identity="twist_ctrl"
  │     └── /mobile_move 서비스 처리
  │
  ├── woosh_rviz_debug.py (서브프로세스)
  │     └── SDK 연결: robot_identity="rviz_debug"
  │     └── /woosh/* 토픽 발행
  │
  ├── RViz (서브프로세스)
  │     └── amcl_debug.rviz 설정 사용
  │
  └── roslaunch amcl.launch launch_rviz:=false (서브프로세스)
        ├── woosh_sensor_bridge: robot_identity="amcl_bridge"
        ├── map_server
        └── amcl
```

> SDK 연결이 총 3개(`twist_ctrl`, `rviz_debug`, `amcl_bridge`) 발생합니다. Woosh TR-200은 복수 WebSocket 연결을 지원하므로 정상 동작합니다.

---

## RViz 시각화

`amcl_debug.rviz` 에서 확인할 수 있는 디스플레이:

| 디스플레이 | 토픽 | 설명 |
|-----------|------|------|
| Map | `/map` | 저장된 점유 격자 맵 |
| LaserScan | `/scan` | 실시간 레이저 스캔 포인트 |
| PoseWithCovariance | `/amcl_pose` | AMCL 추정 위치 + 불확실도 타원 |
| PoseArray | `/particlecloud` | 파티클 분포 (수렴 정도 시각화) |
| Odometry | `/odom` | 합성 오도메트리 궤적 |
| TF | — | map / odom / base_link / laser 프레임 |

### 초기 위치 설정 도구

RViz 툴바:
- **2D Pose Estimate**: 초기 위치 수동 지정 → `/initialpose` 토픽으로 발행
- **2D Nav Goal**: (향후 move_base 연동 시 사용)

---

## 튜닝 가이드

### 파티클 수렴이 느릴 때

1. `max_particles`를 `5000`으로 늘립니다 (CPU 여유가 있는 경우).
2. `update_min_d` / `update_min_a` 값을 줄여 더 자주 업데이트합니다.
3. 로봇을 회전시켜 파티클 분산을 강제합니다.

### 위치 추정이 불안정할 때 (튀는 현상)

1. `laser_sigma_hit`을 `0.1`~`0.15`로 줄여 레이저 모델을 더 엄격하게 만듭니다.
2. `odom_alpha1~4` 값을 낮춰 오도메트리 신뢰도를 높입니다 (단, 실제 드리프트보다 작으면 안 됨).
3. `recovery_alpha_fast`를 `0.05`로 낮춰 과도한 복구 시도를 줄입니다.

### WebSocket 지연으로 TF 오류 발생 시

```
[WARN] Extrapolation Error ... for frame 'base_link'
```

`transform_tolerance`를 `1.0`으로 늘립니다.

### 맵과 실제 환경이 많이 다를 때

`laser_likelihood_max_dist`를 `1.0`으로 줄이고, `laser_z_rand`를 `0.1`로 높여 노이즈 내성을 강화합니다.

### LiDAR 위치 오프셋 조정

실제 LiDAR 장착 위치에 따라 `amcl.launch` 인자를 수정합니다:

```bash
roslaunch woosh_slam_amcl amcl.launch \
  laser_offset_x:=0.1 \
  laser_offset_y:=0.0 \
  laser_offset_z:=0.3 \
  laser_offset_yaw:=0.0
```

---

## 트러블슈팅

### `roslaunch woosh_slam_amcl amcl.launch` 실행 시 패키지를 찾을 수 없음

```bash
# 빌드 후 재소싱
catkin build woosh_slam_amcl
source /root/catkin_ws/devel/setup.bash

# 패키지 인식 확인
rospack find woosh_slam_amcl
```

### `/scan` 토픽이 발행되지 않음

```bash
rostopic list | grep scan
```

`woosh_sensor_bridge` 노드 로그 확인:
```bash
rosnode info /woosh_sensor_bridge
```

SDK 연결 실패인 경우, 로봇 IP와 포트를 확인합니다:
```bash
ping 169.254.128.2
```

### AMCL 파티클이 맵에 퍼져 있고 수렴하지 않음

1. RViz에서 **2D Pose Estimate**로 대략적인 초기 위치를 지정합니다.
2. 로봇을 수동으로 이동/회전시켜 레이저 스캔 패턴을 다양하게 만듭니다.
3. 맵 파일이 현재 환경과 일치하는지 확인합니다.

### 맵 내보내기 실패

```
RuntimeError: 현재 로드된 scene이 없습니다.
```

로봇의 현재 scene을 확인하거나 `_scene_name` 파라미터를 명시적으로 지정합니다:

```bash
rosrun woosh_slam_amcl export_map.py \
  _robot_ip:=169.254.128.2 \
  _scene_name:=my_scene_name
```

---

## 설계 결정 배경

### 왜 합성 오도메트리인가?

Woosh SDK는 바퀴 엔코더를 직접 노출하지 않습니다. `PoseSpeed` 메시지의 `twist.linear` / `twist.angular`는 로봇 내부 추정 속도이며, 이를 시간 적분하여 오도메트리를 합성합니다. 합성 오도메트리는 드리프트가 있지만 AMCL이 이를 보정하므로 실용적입니다.

### 왜 `robot_identity`를 분리하는가?

Woosh SDK는 동일 IP에서 여러 WebSocket 연결을 허용합니다. `robot_identity` 문자열로 각 연결을 구분하여 `woosh_sensor_bridge`(`amcl_bridge`), `woosh_rviz_debug`(`rviz_debug`), `woosh_service_driver`(`twist_ctrl`)가 동시에 연결될 수 있습니다.

### 왜 맵 PNG를 수직으로 뒤집는가?

Woosh SDK의 맵 PNG는 이미지 row 0이 맵의 최대 y 방향(상단)입니다. `map_server`가 읽는 PGM은 파일의 첫 번째 행이 맵의 최소 y 방향(하단)입니다. `export_map.py`에서 y축을 뒤집어 저장하여 좌표계를 일치시킵니다.

### `transform_tolerance: 0.5`로 설정한 이유

Woosh SDK는 WebSocket 기반으로 약 100ms의 지연이 발생합니다. ROS TF의 기본 허용 오차(0.1s)로는 타임스탬프 불일치로 인한 경고가 빈번히 발생하므로 0.5s로 여유 있게 설정했습니다.
