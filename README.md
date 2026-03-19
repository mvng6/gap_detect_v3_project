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
roslaunch woosh_slam slam_gmapping.launch robot_ip:=169.254.128.2
```

### 터미널 3. Docker 진입 + 소싱 + 모바일로봇(TR-200) 연결

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
rosrun woosh_bringup woosh_service_driver.py
```

현재 맵과 로봇을 RViz에서 실시간으로 확인하려면 마지막 명령을 아래처럼 실행합니다.

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash
rosrun woosh_bringup woosh_service_driver.py rviz_on
```

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
├── main_command/          # 두 로봇을 통합 제어하는 최상위 조율 레이어
├── doosan-robot/          # 두산 협동로봇 패키지 (공식 + 커스텀)
│   ├── dsr_control/       # 하드웨어 인터페이스 (C++)
│   ├── dsr_msgs/          # ROS 서비스/메시지 정의 (100+)
│   ├── dsr_description/   # URDF/Xacro 모델
│   ├── dsr_launcher/      # 런치 파일
│   ├── dsr_gazebo/        # Gazebo 시뮬레이션
│   ├── dsr_example/       # 예제 스크립트
│   └── moveit_config_*/   # MoveIt 설정 (로봇 모델별)
└── TR-200/                # Woosh 모바일로봇 패키지
    ├── woosh_robot_py/    # WebSocket SDK 래퍼
    ├── woosh_bringup/     # ROS 노드 + 런치 파일
    ├── woosh_msgs/        # 커스텀 서비스 정의
    └── woosh_utils/       # 유틸리티 (배터리 상태 등)
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

- [x] Docker 개발 환경 구축
- [x] 두산 협동로봇 연결 및 기초 구동
- [x] Woosh 모바일로봇 연결 및 기초 구동
- [x] RViz 디버그 시각화
- [x] 두 로봇 통합 제어 기초 프레임
- [ ] SLAM (지도 생성)
- [ ] 자율 내비게이션 (경로 계획)
- [ ] 갭 감지 알고리즘 개발
- [ ] 협동 작업 시나리오 구현

---

## 라이선스

- 커스텀 패키지 (`main_command`, `TR-200/*`): Proprietary
- 두산 로봇 패키지 (`doosan-robot/*`): BSD / Apache 2.0
