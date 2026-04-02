# Gap Detection V3

두산 협동로봇(A0912)과 Woosh TR-200 모바일 로봇을 통합하여 갭 감지(Gap Detection) 작업을 수행하는 ROS1 Noetic 기반 시스템입니다.

**유지보수**: LDJ (djlee2@katech.re.kr) · KATECH  
**환경**: Docker (ROS1 Noetic) · Ubuntu 24.04 NUC 호스트

---

## 목차

1. [시스템 구성](#1-시스템-구성)
2. [개발 환경 설정](#2-개발-환경-설정)
3. [빌드](#3-빌드)
4. [빠른 시작](#4-빠른-시작)
5. [모바일 로봇 실행 옵션](#5-모바일-로봇-실행-옵션)
6. [지도 생성 및 준비](#6-지도-생성-및-준비)
7. [패키지 구조](#7-패키지-구조)
8. [ROS 서비스 및 연결 정보](#8-ros-서비스-및-연결-정보)
9. [개발 현황](#9-개발-현황)
10. [라이선스](#10-라이선스)

---

## 1. 시스템 구성

| 구성 요소 | 모델 | 통신 |
|-----------|------|------|
| 협동로봇 (Arm) | Doosan A0912 | TCP 192.168.137.100:12345 |
| 모바일 로봇 | Woosh TR-200 | WebSocket 169.254.128.2:5480 (protobuf) |
| 운영 환경 | Docker (ROS1 Noetic) | — |

```
main_command
├── /dsr01a0912/motion/move_joint ──► dsr_control (C++) ──► Doosan A0912
└── /mobile_move ──────────────────► woosh_service_driver (Python) ──► Woosh TR-200
```

---

## 2. 개발 환경 설정

> 모든 ROS 명령은 Docker 컨테이너 내부에서 실행합니다.

```bash
# 호스트: X11 접근 허용 및 컨테이너 시작 (1회)
xhost +local:docker
docker-compose -f docker-compose.noetic_integration.yml up -d

# 컨테이너 진입
docker exec -it noetic_robot_system_ws bash
```

---

## 3. 빌드

컨테이너 내부 `/root/catkin_ws`에서 실행합니다.

```bash
cd /root/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

---

## 4. 빠른 시작

터미널 4개를 열어 순서대로 실행합니다. 각 터미널에서 `docker exec -it noetic_robot_system_ws bash && source /root/catkin_ws/devel/setup.bash`로 진입합니다.

| 터미널 | 명령 | 역할 |
|--------|------|------|
| T1 | `roscore` | ROS 마스터 |
| T2 | `roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=real server_ip:=192.168.137.100` | 두산 협동로봇 연결 |
| T3 | `roslaunch woosh_bringup woosh_navigation_system.launch [args]` | 모바일 로봇 + 내비게이션 스택 |
| T4 | `rosrun main_command main_system_operation.py` | 통합 제어 실행 |

### T3 주요 실행 예시

```bash
# 기본 (WebSocket 연결 + /mobile_move 서비스만)
roslaunch woosh_bringup woosh_navigation_system.launch

# AMCL + move_base 자율 내비게이션
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=amcl navigation_mode:=move_base \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# Cartographer 고정 맵 + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  localization_mode:=carto_fix navigation_mode:=move_base \
  state_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# GMapping SLAM + move_base
roslaunch woosh_bringup woosh_navigation_system.launch \
  slam_mode:=gmapping navigation_mode:=move_base
```

### `woosh_navigation_system.launch` 주요 인자

| 인자 | 기본값 | 옵션 |
|------|--------|------|
| `slam_mode` | `none` | `none` \| `gmapping` \| `cartographer` |
| `localization_mode` | `none` | `none` \| `amcl` \| `carto_fix` \| `carto_nonfix` |
| `navigation_mode` | `none` | `none` \| `costmap` \| `move_base` |
| `map_file` | `""` | `.yaml` 맵 파일 경로 (AMCL/costmap 시 필요) |
| `state_file` | `""` | `.pbstream` 파일 경로 (Cartographer loc 시 필요) |
| `launch_rviz` | `true` | RViz 자동 실행 여부 |
| `global_planner_plugin` | `navfn/NavfnROS` | 글로벌 플래너 플러그인 |
| `local_planner_plugin` | `dwa_local_planner/DWAPlannerROS` | 로컬 플래너 플러그인 |

---

## 5. 모바일 로봇 실행 옵션

### 정식 방식 (권장) — `woosh_navigation_system.launch`

위 [4절](#4-빠른-시작) 참고.

### 레거시 방식 — `rosrun woosh_service_driver.py`

| 명령 | 설명 |
|------|------|
| `rosrun woosh_bringup woosh_service_driver.py` | `/mobile_move` 서비스만 |
| `... rviz_on` | + RViz |
| `... gmap` | + GMapping SLAM |
| `... carto_map` | + Cartographer SLAM |
| `... carto_loc_fix` | + Cartographer 로컬라이제이션 (고정 맵) |
| `... amcl map_file:=<path>` | + AMCL 로컬라이제이션 |
| `... amcl move_base_on map_file:=<path>` | + AMCL + move_base |
| `... carto_loc_fix move_base_on state_file:=<path>` | + Cartographer(fix) + move_base |

### 두산 로봇 단독 실행

```bash
# 가상 모드
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=virtual

# Gazebo 시뮬레이션
roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=a0912 mode:=virtual

# 에뮬레이터 (실제 로봇 없이 개발)
cd src/doosan-robot/doosan_robot && ./install_emulator.sh   # 최초 1회
cd src/doosan-robot/common/bin/DRCF && ./run_drcf.sh        # 127.0.0.1:12345
```

---

## 6. 지도 생성 및 준비

로컬라이제이션 전에 맵 파일(`.pgm` + `.yaml`)을 준비합니다.

### GMapping으로 생성

```bash
rosrun woosh_bringup woosh_service_driver.py gmap   # 로봇 조작하며 영역 탐색
roslaunch woosh_slam_gmapping save_map.launch map_name:=woosh_map
```

### Cartographer로 생성

```bash
rosrun woosh_bringup woosh_service_driver.py carto_map
roslaunch woosh_slam_cartographer save_map.launch map_name:=woosh_map
```

### 로봇에서 기존 맵 내보내기

```bash
rosrun woosh_slam_amcl export_map.py \
  _robot_ip:=169.254.128.2 \
  _output_dir:=/root/catkin_ws/src/TR-200/woosh_slam/maps \
  _map_name:=woosh_map
```

저장 위치: `/root/catkin_ws/src/TR-200/woosh_slam/maps/`

---

## 7. 패키지 구조

```
src/
├── main_command/                   # 통합 제어 최상위 노드 (Python)
├── cartographer_src/               # Git 서브모듈: cartographer, cartographer_ros
├── doosan-robot/                   # 두산 협동로봇 패키지
│   ├── dsr_control/                # 하드웨어 인터페이스 (C++)
│   ├── dsr_msgs/                   # ROS 서비스/메시지 정의 (100+)
│   ├── dsr_description/            # URDF/Xacro 모델
│   ├── dsr_launcher/               # 런치 파일
│   ├── dsr_gazebo/                 # Gazebo 시뮬레이션
│   └── moveit_config_*/            # MoveIt 설정 (모델별)
└── TR-200/                         # Woosh 모바일 로봇 패키지
    ├── woosh_robot_py/             # WebSocket SDK 래퍼
    ├── woosh_sensor_bridge/        # 센서 브릿지 (/scan, /odom, TF)
    ├── woosh_bringup/              # 주요 노드 + 런치 파일
    │   ├── scripts/                # woosh_service_driver.py, woosh_rviz_debug.py
    │   └── launch/                 # woosh_navigation_system.launch (정식 진입점)
    ├── woosh_msgs/                 # 커스텀 서비스 (MoveMobile.srv)
    ├── woosh_navigation/
    │   ├── AMCL/woosh_slam_amcl/   # AMCL 로컬라이제이션
    │   ├── Costmap/woosh_costmap/  # Global Costmap standalone
    │   ├── MoveBase/woosh_navigation_mb/  # move_base 자율 내비게이션
    │   └── maps/                   # 맵 파일 (.pgm + .yaml)
    └── woosh_slam/
        ├── GMapping/woosh_slam_gmapping/
        └── Cartographer/woosh_slam_cartographer/
```

### 주요 소스 파일

| 파일 | 역할 |
|------|------|
| `src/main_command/scripts/main_system_operation.py` | 두 로봇 통합 제어 시퀀스 |
| `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py` | `/mobile_move` 서비스 + 내비게이션 스택 오케스트레이터 |
| `src/TR-200/woosh_sensor_bridge/scripts/woosh_sensor_bridge.py` | WebSocket 센서 → `/scan`, `/odom`, TF 변환 |
| `src/TR-200/woosh_robot_py/woosh_robot.py` | Woosh WebSocket SDK |
| `src/doosan-robot/dsr_control/src/dsr_hw_interface.cpp` | 두산 하드웨어 추상화 레이어 |

---

## 8. ROS 서비스 및 연결 정보

### 주요 ROS 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/mobile_move` | `woosh_msgs/MoveMobile` | 직선 이동 (`distance` m, 음수=후진) |
| `/dsr01a0912/motion/move_joint` | `dsr_msgs/MoveJoint` | 관절 공간 이동 |
| `/dsr01a0912/motion/move_tcp` | `dsr_msgs/MoveTcp` | 직교 공간 이동 |

### 로봇 연결 정보

| 로봇 | IP | 포트 | 프로토콜 |
|------|----|------|----------|
| Doosan A0912 (실제) | 192.168.137.100 | 12345 | TCP |
| Doosan (에뮬레이터) | 127.0.0.1 | 12345 | TCP |
| Woosh TR-200 | 169.254.128.2 | 5480 | WebSocket (protobuf) |

### 지원 두산 모델

a0509, **a0912**, e0509, h2017, h2515, m0609, m0617, m1013, m1509

---

## 9. 개발 현황

### 완료

- [x] Docker 개발 환경 · 두 로봇 연결 · RViz 시각화
- [x] 센서 토픽 (`/scan`, `/odom`, TF) 정리 및 안정화
- [x] Twist 적분 기반 합성 오도메트리 (`woosh_sensor_bridge.py`)
- [x] GMapping / Cartographer SLAM
- [x] AMCL · Cartographer 로컬라이제이션 (fix / nonfix)
- [x] Global/Local Costmap 구성
- [x] `move_base` 자율 내비게이션 패키지 구현
- [x] `woosh_navigation_system.launch` — Launch-Parameter-Centered 정식 진입점 (2026-04-02)
- [x] SLAM + move_base 동시 실행 지원

### 진행 중 / 예정

- [ ] 자율주행 실기 테스트 (목표점 이동, 장애물 회피)
- [ ] 오도메트리 정확도 측정 및 공분산 파라미터 튜닝
- [ ] AMCL 파라미터 튜닝 · 재로컬라이제이션 절차 검증
- [ ] 비상 정지(E-stop) · 통신 두절 시 fail-safe
- [ ] 두산 협동로봇 + 모바일 로봇 통합 미션 시퀀스
- [ ] 갭 감지 알고리즘 개발 및 통합

---

## 10. 라이선스

- 커스텀 패키지 (`main_command`, `TR-200/*`): Proprietary
- 두산 로봇 패키지 (`doosan-robot/*`): BSD / Apache 2.0
