# Gap Detection V3 - 협동로봇 + 모바일로봇 통합 시스템

두산 협동로봇(Doosan)과 Woosh TR-200 모바일 로봇을 통합하여 갭 감지(Gap Detection) 작업을 수행하는 ROS1 Noetic 기반 로봇 시스템입니다.

현재는 각 로봇의 연결 및 기초 구동 환경이 구축된 상태이며, 이후 SLAM, 자율 내비게이션, 갭 감지 알고리즘 등을 단계적으로 추가할 예정입니다.

## 통합 시스템 빠른 실행

아래 순서대로 `터미널 1 → 터미널 2 → 터미널 3 → 터미널 4`를 각각 별도로 열어 실행하면 통합 시스템을 구동할 수 있습니다. 모든 명령은 `noetic_robot_system_ws` Docker 컨테이너 기준입니다.

### 사전 준비 (호스트에서 1회)

컨테이너가 아직 실행 중이 아니라면 먼저 아래 명령을 실행합니다.

```bash
xhost +local:docker
docker-compose -f docker-compose.noetic_integration.yml up -d
```

### 터미널 1. Docker 진입 + 소싱 + `roscore`

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
roscore
```

### 터미널 2. Docker 진입 + 소싱 + 두산로봇 연결

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=real server_ip:=192.168.137.100
```

### 터미널 3. Docker 진입 + 소싱 + 모바일로봇(TR-200) 연결

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
rosrun woosh_bringup woosh_service_driver.py
```

센서 데이터와 로봇 상태를 RViz에서 실시간으로 확인하려면:

```bash
rosrun woosh_bringup woosh_service_driver.py rviz_on
```

지도 생성(GMapping SLAM)과 함께 실행하려면:

```bash
rosrun woosh_bringup woosh_service_driver.py gmap
```

지도 생성(Cartographer SLAM)과 함께 실행하려면:

```bash
rosrun woosh_bringup woosh_service_driver.py carto_map
```

Cartographer 로컬라이제이션(고정 맵)과 함께 실행하려면:

```bash
rosrun woosh_bringup woosh_service_driver.py carto_loc_fix
```

Cartographer 로컬라이제이션(서브맵 업데이트 포함)과 함께 실행하려면:

```bash
rosrun woosh_bringup woosh_service_driver.py carto_loc_nonfix
```

> 생성한 지도 저장 방법은 아래 `실행 방법 > 지도 생성 및 지도 파일 준비` 섹션을 참조하세요.

로컬라이제이션(AMCL, 위치 추정)과 함께 실행하려면:

```bash
# 기본 맵 파일 사용
rosrun woosh_bringup woosh_service_driver.py amcl

# 맵 파일 경로를 직접 지정하는 경우
rosrun woosh_bringup woosh_service_driver.py amcl map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/my_map.yaml
```

> 로컬라이제이션에는 맵 파일이 필요합니다. 맵 생성 또는 맵 내보내기는 아래 `실행 방법 > 지도 생성 및 지도 파일 준비` 섹션을 먼저 확인하세요.

### 터미널 4. Docker 진입 + 소싱 + 통합 동작 실행

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
rosrun main_command main_system_operation.py
```

---

## 시스템 구성

| 구성 요소 | 모델 | 통신 방식 |
|-----------|------|-----------|
| 협동로봇 (Arm) | Doosan A0912 | TCP/Serial (포트 12345) |
| 모바일로봇 | Woosh TR-200 | WebSocket (protobuf) |
| 운영 환경 | Docker (ROS1 Noetic) | — |
| 호스트 OS | Ubuntu 24.04 (NUC) | — |

### 아키텍처 개요

```
main_command (통합 제어 노드)
│
├── /dsr01a0912/motion/move_joint ──► dsr_control (C++) ──► Doosan A0912
│                                       192.168.137.100:12345
│
└── /mobile_move ──────────────────► woosh_service_driver (Python) ──► Woosh TR-200
                                        169.254.128.2:5480 (WebSocket)
```

---

## 패키지 구조

```
src/
├── main_command/              # 두 로봇을 통합 제어하는 최상위 조율 레이어
├── doosan-robot/              # 두산 협동로봇 패키지 (공식 + 커스텀)
│   ├── dsr_control/           # 하드웨어 인터페이스 (C++)
│   ├── dsr_msgs/              # ROS 서비스/메시지 정의 (100+)
│   ├── dsr_description/       # URDF/Xacro 모델
│   ├── dsr_launcher/          # 런치 파일
│   ├── dsr_gazebo/            # Gazebo 시뮬레이션
│   ├── dsr_example/           # 예제 스크립트
│   └── moveit_config_*/       # MoveIt 설정 (로봇 모델별)
└── TR-200/                    # Woosh 모바일로봇 패키지
    ├── woosh_robot_py/        # WebSocket SDK 래퍼
    ├── woosh_sensor_bridge/   # SDK 센서 브릿지 (/scan, /odom, TF)
    │   └── scripts/           # woosh_sensor_bridge.py
    ├── woosh_bringup/         # ROS 노드 + 런치 파일
    │   ├── scripts/           # woosh_service_driver.py, woosh_rviz_debug.py
    │   ├── launch/            # woosh_rviz_debug.launch
    │   └── rviz/              # woosh_rviz_debug.rviz
    ├── woosh_msgs/            # 커스텀 서비스 정의 (MoveMobile.srv)
    ├── woosh_utils/           # 유틸리티 (배터리 상태 출력)
    ├── woosh_navigation/      # 네비게이션 패키지 모음
    │   ├── AMCL/              # woosh_slam_amcl — 맵 기반 위치 추정
    │   │   ├── scripts/       # export_map.py
    │   │   ├── launch/        # amcl.launch
    │   │   ├── config/        # amcl_params.yaml
    │   │   ├── rviz/          # amcl_debug.rviz
    │   │   └── docs/          # amcl_guide.md
    │   └── maps/              # 맵 파일 (.pgm + .yaml)
    └── woosh_slam/            # SLAM 패키지 모음 (지도 생성)
        ├── GMapping/          # GMapping 기반 SLAM
        │   └── woosh_slam_gmapping/  # ROS 패키지
        └── Cartographer/      # Cartographer 기반 SLAM
            └── woosh_slam_cartographer/  # ROS 패키지
```

---

## 개발 환경 설정

### 사전 요구사항

- Docker, Docker Compose
- X11 디스플레이 (RViz/Gazebo GUI용)

### 1. 컨테이너 빌드 및 실행

```bash
# X11 접근 허용
xhost +local:docker

# 컨테이너 시작
docker-compose -f docker-compose.noetic_integration.yml up -d

# 컨테이너 진입
docker exec -it noetic_robot_system_ws bash
```

### 2. ROS 워크스페이스 빌드 (컨테이너 내부)

```bash
cd /root/catkin_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build

# 환경 설정
source devel/setup.bash
```

---

## 실행 방법

### 모바일 로봇 (Woosh TR-200)

```bash
# RViz 디버그 모드 (시각화 + 로봇 상태 확인)
roslaunch woosh_bringup woosh_rviz_debug.launch robot_ip:=169.254.128.2
```

### 지도 생성 및 지도 파일 준비

로컬라이제이션 전에 먼저 사용할 맵 파일(`.pgm` + `.yaml`)을 준비합니다.
아래 세 가지 방법 중 하나를 선택하면 됩니다.

#### 1. GMapping으로 새 지도 생성

```bash
# 1단계-A: GMapping 스택 단독 실행 (RViz 포함)
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2

# 1단계-B: woosh_service_driver 와 통합 실행 (권장)
rosrun woosh_bringup woosh_service_driver.py gmap

# 2단계: 로봇을 탐색 영역 전체에 걸쳐 이동시킵니다.
# /map 발행 확인: rostopic hz /map

# 3단계: 지도 저장 (별도 터미널, GMapping 실행 중인 상태)
roslaunch woosh_slam_gmapping save_map.launch map_name:=woosh_map
# 저장 위치: /root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.{pgm,yaml}
```

#### 2. Cartographer로 새 지도 생성

```bash
# 1단계-A: Cartographer 스택 단독 실행 (RViz 포함)
roslaunch woosh_slam_cartographer cartographer.launch robot_ip:=169.254.128.2

# 1단계-B: woosh_service_driver 와 통합 실행 (권장)
rosrun woosh_bringup woosh_service_driver.py carto_map

# 2단계: 로봇을 탐색 영역 전체에 걸쳐 이동시킵니다.

# 3단계: 지도 저장 (별도 터미널, Cartographer 실행 중인 상태)
roslaunch woosh_slam_cartographer save_map.launch map_name:=woosh_map
# 저장 위치: /root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.{pgm,yaml}
```

#### 3. 로봇에 저장된 기존 맵 내보내기

```bash
rosrun woosh_slam_amcl export_map.py \
  _robot_ip:=169.254.128.2 \
  _output_dir:=/root/catkin_ws/src/TR-200/woosh_slam/maps \
  _map_name:=woosh_map
```

> 준비된 지도 파일은 아래 `AMCL 로컬리제이션` 섹션의 `map_file` 인자로 바로 사용할 수 있습니다.
> 새 지도 생성 절차는 [`src/TR-200/woosh_slam/GMapping/woosh_slam_gmapping/docs/gmapping_guide.md`](src/TR-200/woosh_slam/GMapping/woosh_slam_gmapping/docs/gmapping_guide.md)를 참조하세요.

### AMCL 로컬리제이션 (woosh_slam_amcl)

AMCL(Adaptive Monte Carlo Localization)은 이미 준비된 맵 위에서 로봇의 위치를 실시간으로 추정합니다.

```bash
# 1단계-A: AMCL 스택 단독 실행 (RViz 포함)
roslaunch woosh_slam_amcl amcl.launch \
  robot_ip:=169.254.128.2 \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml

# 1단계-B: woosh_service_driver 와 통합 실행 (권장)
rosrun woosh_bringup woosh_service_driver.py amcl \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml
```

> 자세한 내용은 [`src/TR-200/woosh_navigation/AMCL/docs/amcl_guide.md`](src/TR-200/woosh_navigation/AMCL/docs/amcl_guide.md)를 참조하세요.

### 두산 협동로봇

```bash
# 가상 모드 (실제 로봇 없이 테스트)
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=virtual

# 실제 로봇 연결
roslaunch dsr_launcher single_robot_rviz.launch model:=a0912 mode:=real server_ip:=192.168.137.100

# Gazebo 시뮬레이션
roslaunch dsr_launcher single_robot_rviz_gazebo.launch model:=a0912 mode:=virtual
```

### 통합 시스템 실행

```bash
# 두 로봇 모두 연결된 상태에서 실행
rosrun main_command main_system_operation.py
```

### 두산 에뮬레이터 (실제 로봇 없이 개발)

```bash
# 에뮬레이터 설치 (최초 1회)
cd src/doosan-robot/doosan_robot && ./install_emulator.sh

# 에뮬레이터 실행 (127.0.0.1:12345)
cd src/doosan-robot/common/bin/DRCF && ./run_drcf.sh
```

---

## 주요 ROS 서비스

| 서비스명 | 메시지 타입 | 설명 |
|----------|------------|------|
| `/mobile_move` | `woosh_msgs/MoveMobile` | 모바일 로봇 직선 이동 (distance: m, 음수=후진) |
| `/dsr01a0912/motion/move_joint` | `dsr_msgs/MoveJoint` | 관절 공간 이동 |
| `/dsr01a0912/motion/move_tcp` | `dsr_msgs/MoveTcp` | 직교 공간 이동 |

---

## 로봇 연결 정보

| 로봇 | IP 주소 | 포트 | 프로토콜 |
|------|---------|------|----------|
| Doosan A0912 (실제) | 192.168.137.100 | 12345 | TCP/Serial |
| Doosan (에뮬레이터) | 127.0.0.1 | 12345 | TCP/Serial |
| Woosh TR-200 | 169.254.128.2 | 5480 | WebSocket (protobuf) |

---

## 지원 두산 로봇 모델

a0509, **a0912**, e0509, h2017, h2515, m0609, m0617, m1013, m1509

---

## 개발 현황 및 로드맵

### 1. 개발/통합 기반
- [x] Docker 개발 환경 구축
- [x] 두산 협동로봇 연결 및 기초 구동
- [x] Woosh 모바일로봇 연결 및 기초 구동
- [x] RViz 디버그 시각화
- [x] 두 로봇 통합 제어 기초 프레임

### 2. 모바일로봇 기본 주행 기반
- [x] 모바일 로봇 센서 토픽 정리 (`/scan`, `/odom`, `/tf` 등)
  - `/scan`, `/odom`, TF(`odom → base_link`): **`woosh_sensor_bridge.py`가 유일한 발행 주체**
  - 모든 모드에서 동일하게 SDK `scanner_data_sub` 구독 → `sensor_msgs/LaserScan` 변환 → `/scan` 발행
  - `woosh_service_driver.py`가 항상 sensor_bridge를 subprocess로 기동 (모드 무관)
  - SLAM/AMCL launch 파일은 `launch_sensor_bridge` arg(default: `true`)로 중복 기동 방지
    - 단독 실행 (`roslaunch gmapping.launch`): sensor_bridge가 launch 파일 내에서 기동
    - service_driver 통합 실행 (`woosh_service_driver.py gmap`): service_driver가 sensor_bridge 기동 → launch 파일에 `launch_sensor_bridge:=false` 전달
- [x] LiDAR / IMU / 바퀴 엔코더 입력 상태 점검
  - [x] LiDAR: `woosh_sensor_bridge.py` → SDK `scanner_data_sub` 구독 → `/scan` 발행 (`frame_id: laser`)
  - [ ] IMU: Woosh SDK 미제공 — 미구현
  - [ ] 바퀴 엔코더: 직접 엔코더 없음 — twist 적분 기반 합성 오도메트리로 대체 중
- [x] `base_link`, `laser`, `odom`, `map` 프레임 구조 점검
  - `odom → base_link`: `woosh_sensor_bridge.py`, `base_link → laser`: static_transform_publisher (z=0.25m), `map → odom`: SLAM 노드
- [x] TF tree 안정화 (`map -> odom -> base_link -> sensor`)
  - `transform_tolerance: 0.5s` 설정으로 WebSocket 통신 지연 대응
- [ ] 로봇 footprint / radius 정의
  - [x] RViz 시각화용 정의 완료 (`woosh_rviz_debug.py`: 0.65m × 0.45m)
  - [ ] Costmap / move_base에 footprint 미적용 — 자율주행 경로 계획 시 별도 설정 필요
- [ ] 속도 명령(`/cmd_vel`)과 실제 모바일로봇 구동 연동 확인
  - `/cmd_vel` 토픽 구독 미구현 → move_base(ROS navigation stack) 통합 불가
  - 현재는 `/mobile_move` 서비스(고정 거리 직진/후진)만 지원, 제자리 회전 미지원

### 3. 오도메트리 / 상태추정
- [x] Twist 적분 기반 합성 오도메트리 구현 (`woosh_sensor_bridge.py`)
  - Woosh SDK가 바퀴 엔코더를 미제공 → `PoseSpeed.twist`(linear, angular)를 dt 적분하는 차동 구동 모델로 대체
  - 포즈 공분산(x: 0.05, y: 0.05, yaw: 0.1) 및 속도 공분산(vx: 0.001, ω: 0.005) 기본값 설정
- [x] `PoseSpeed.pose` 필드 확인 (로봇 내부 SLAM 기반 위치 추정값)
  - `Pose2D { x, y, theta }` — 로봇이 LiDAR + 내부 지도로 스스로 추정한 절대 위치
  - `map_id != 0` 일 때만 유효 (지도 로드 상태 필수)
  - `woosh_rviz_debug.py` 에서 시각화에 활용 중 / `woosh_sensor_bridge.py` 에서는 미활용 (개선 여지)
  - 상세 분석 및 예제: [`src/TR-200/woosh_robot_py/docs/pose_speed_pose_guide.md`](src/TR-200/woosh_robot_py/docs/pose_speed_pose_guide.md)
- [x] `PoseSpeed.mileage` 필드 확인 (누적 주행 거리)
  - `int` 타입, 로봇 생산 이후 전체 누적 주행 거리 (단위: mm 추정)
  - 오도메트리 소스로는 부적합 — 유지보수 주기 계산·테스트 로그 기록 용도
- [ ] Wheel odometry 정확도 점검
  - 실측 이동 거리/각도 vs `/odom` 누적값 비교 (기준 거리 마킹 후 반복 측정)
- [ ] 직진/회전 시 odom 누적 오차 측정
  - 정해진 경로(예: 1 m 직진, 90° 회전)를 반복 후 복귀 오차 기록
- [ ] IMU 융합 검토 (`robot_localization`, EKF)
  - Woosh SDK IMU 미제공 — 외부 IMU 모듈 추가 장착 시 재검토 필요
- [ ] `/odom` 기반 RViz 이동 궤적 검증
  - RViz `Odometry` 디스플레이 또는 `Path` 플러그인으로 실제 이동 궤적과 비교
- [ ] `PoseSpeed.pose` 직접 활용 검토 (`woosh_sensor_bridge.py` 개선)
  - twist 적분 대신 `pose.x / pose.y / pose.theta` 를 `/odom` 소스로 사용 시 오차 누적 없음
  - 전제 조건: `map_id != 0` (SLAM 지도 로드 상태) 보장 필요
- [ ] 공분산 파라미터 튜닝
  - 실측 오차를 반영하여 `woosh_sensor_bridge.py` 내 포즈·속도 공분산 값 보정

### 4. 지도 생성(SLAM)
- [x] GMapping SLAM (온라인 지도 생성)
- [x] Cartographer SLAM (온라인 지도 생성)
- [ ] GMapping 지도 저장 및 재사용 검증
- [ ] Cartographer 지도 저장 및 재사용 검증
- [ ] GMapping vs Cartographer 맵 품질 비교
- [ ] 최종 운영용 SLAM 방식 1종 선정

### 5. 맵 관리
- [ ] 저장된 맵(`.pgm`, `.yaml`) 관리 규칙 정리
- [ ] `map_server` 기반 정적 맵 로드 확인
- [ ] 맵 버전별 테스트 환경 구분
- [ ] 운영용 기준 맵 1종 확정

### 6. 로컬라이제이션
- [x] AMCL 로컬리제이션 (맵 기반 위치 추정)
- [x] Cartographer 로컬리제이션 — fix 모드 (고정 맵, AMCL 유사)
- [x] Cartographer 로컬리제이션 — nonfix 모드 (서브맵 업데이트 포함)
- [ ] 초기 자세 설정 절차 정리 (`2D Pose Estimate`)
- [ ] AMCL 파라미터 튜닝
- [ ] AMCL 위치 오차 반복 측정
- [ ] 재시작 후 재로컬라이제이션 절차 검증

### 7. Costmap 구성
- [ ] Global Costmap 구성
- [ ] Local Costmap 구성
- [ ] Static Layer 구성
- [ ] Obstacle Layer 구성
- [ ] Inflation Layer 구성
- [ ] 장애물 반영 / 제거 동작 확인
- [ ] 동적 장애물 대응 여부 점검

### 8. 경로 계획 및 자율주행
- [ ] 자율 내비게이션 (`move_base`)
- [ ] Global Planner 설정
- [ ] Local Planner 설정 (DWA 등)
- [ ] 목표점 1개 이동 성공
- [ ] 복수 waypoint 이동 성공
- [ ] 주행 중 장애물 회피 동작 검증
- [ ] 목표 도달 허용 오차(x, y, yaw) 튜닝

### 9. 주행 안정화 / 복구 동작
- [ ] Goal 실패 시 재시도 로직
- [ ] 회전 recovery 동작 확인
- [ ] 경로 재계획(replan) 동작 확인
- [ ] localization 불안정 시 예외 처리
- [ ] 센서 끊김 / 통신 오류 시 안전 정지 처리

### 10. 안전 기능
- [ ] 비상 정지(E-stop) 절차 정리
- [ ] 최대 속도 / 가속도 제한 적용
- [ ] 사람/장애물 근접 시 감속 또는 정지 정책 정의
- [ ] 통신 두절 시 fail-safe 정지
- [ ] 테스트 구역 안전 운영 규칙 문서화

### 11. 상위 미션 / 통합 제어
- [ ] 목적지 지정 → 이동 → 정지 시나리오 구현
- [ ] 모바일로봇 도착 후 두산 협동로봇 작업 트리거
- [ ] 작업 완료 후 다음 위치 이동 시나리오 구현
- [ ] 상태 머신(FSM) 또는 시퀀스 제어 구조 정리
- [ ] 예외 상황(미도착, 작업 실패, 재시도) 처리

### 12. 비전/작업 응용
- [ ] 갭 감지 알고리즘 개발
- [ ] 비전 센서 장착 위치 및 캘리브레이션 검토
- [ ] 모바일로봇 정지 위치 정밀도 검증
- [ ] 협동로봇 + 비전 + 모바일로봇 작업 순서 통합
- [ ] 협동 작업 시나리오 구현

### 13. 검증 및 문서화
- [ ] rosbag 기록 체계 정리
- [ ] 주요 토픽/TF 디버깅 체크리스트 문서화
- [ ] 반복 실험 시나리오 작성
- [ ] 성능 평가 지표 정의
  - [ ] localization 오차
  - [ ] goal 도달 성공률
  - [ ] 주행 시간
  - [ ] 장애물 회피 성공률
- [ ] 최종 데모 시나리오 문서화
---

## 라이선스

- 커스텀 패키지 (`main_command`, `TR-200/*`): Proprietary
- 두산 로봇 패키지 (`doosan-robot/*`): BSD / Apache 2.0
