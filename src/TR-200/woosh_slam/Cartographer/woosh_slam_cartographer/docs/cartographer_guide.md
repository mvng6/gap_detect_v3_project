# woosh_slam_cartographer — 사용 가이드

## 개요

Woosh TR-200 모바일 로봇을 위한 **Google Cartographer** 기반 2D SLAM 패키지입니다.

- **입력**: `/scan` (LiDAR), `/odom` (합성 오도메트리), TF
- **출력**: `/map` (OccupancyGrid), TF(`map→odom`)
- **센서 브릿지**: `woosh_slam_amcl` 패키지의 `woosh_sensor_bridge.py` 재사용
- **SDK 연결 식별자**: `cartographer_bridge` (AMCL/GMapping과 충돌 없음)

---

## 빠른 시작

### 1. 패키지 빌드

```bash
# Docker 컨테이너 안에서
cd /root/catkin_ws
catkin build woosh_slam_cartographer
source devel/setup.bash
```

### 2. Cartographer SLAM 실행

```bash
# 방법 A: woosh_service_driver 통합 실행 (권장)
rosrun woosh_bringup woosh_service_driver.py carto

# 방법 B: 직접 launch 실행
roslaunch woosh_slam_cartographer cartographer.launch robot_ip:=169.254.128.2

# RViz 없이 실행
roslaunch woosh_slam_cartographer cartographer.launch launch_rviz:=false
```

### 3. 로봇 이동 (별도 터미널)

```bash
# 서비스 드라이버가 실행 중일 때
rosservice call /mobile_move "{distance: 1.0}"   # 1m 전진
rosservice call /mobile_move "{distance: -1.0}"  # 1m 후진
```

### 4. 지도 저장

```bash
# Cartographer가 실행 중인 상태에서 별도 터미널에서 실행
roslaunch woosh_slam_cartographer save_map.launch map_name:=my_map

# 저장 결과
# /root/catkin_ws/src/TR-200/woosh_navigation/maps/my_map.pgm
# /root/catkin_ws/src/TR-200/woosh_navigation/maps/my_map.yaml
```

### 5. 저장된 지도로 AMCL 로컬리제이션 실행

```bash
# Cartographer로 생성한 지도를 AMCL에서 즉시 사용 가능
rosrun woosh_bringup woosh_service_driver.py amcl \
  map_file:=/root/catkin_ws/src/TR-200/woosh_navigation/maps/my_map.yaml
```

---

## TF 트리

```
map (고정 프레임)
 └── odom    ← cartographer_node 발행 (map→odom, 루프 클로저 보정)
      └── base_link  ← woosh_sensor_bridge 발행 (odom→base_link, twist 적분)
           └── laser  ← static_transform_publisher 발행 (z=0.25m)
```

---

## 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/scan` | sensor_msgs/LaserScan | 입력 | LiDAR 스캔 (woosh_sensor_bridge 발행) |
| `/odom` | nav_msgs/Odometry | 입력 | 합성 오도메트리 (woosh_sensor_bridge 발행) |
| `/map` | nav_msgs/OccupancyGrid | 출력 | 실시간 점유 격자 지도 (0.03m/cell) |
| `/submap_list` | cartographer_ros_msgs/SubmapList | 출력 | Cartographer 서브맵 목록 |

---

## GMapping과의 차이점

| 항목 | GMapping | Cartographer |
|------|---------|-------------|
| 알고리즘 | Rao-Blackwellized 파티클 필터 | Sparse Pose Graph + 루프 클로저 |
| 루프 클로저 | 미지원 | 지원 (넓은 공간에서 정확도 향상) |
| 설정 파일 | YAML (`gmapping_params.yaml`) | LUA (`cartographer_2d.lua`) |
| 메모리 사용 | 파티클 수에 비례 | 서브맵 수에 비례 |
| CPU 사용 | 낮음 | 루프 클로저 최적화 시 높음 |
| 맵 품질 | 좁은 공간에서 우수 | 넓은 공간, 루프 포함 환경에서 우수 |

---

## 주요 파라미터 조정

### config/cartographer_2d.lua

```lua
-- 레이저 유효 범위 (m)
TRAJECTORY_BUILDER_2D.max_range = 8.0

-- 루프 클로저 최적화 빈도 (노드 수)
-- 낮을수록 자주 최적화 (CPU 증가)
POSE_GRAPH.optimize_every_n_nodes = 90

-- 실시간 스캔 매칭 활성화 (오도메트리 불량 시 필수)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

---

## 문제 해결

### `/map` 토픽이 발행되지 않는 경우

```bash
# cartographer_occupancy_grid_node 실행 확인
rosnode list | grep cartographer

# 서브맵 목록 확인
rostopic echo /submap_list --noarr
```

### TF 오류가 발생하는 경우

```bash
# TF 트리 시각화
rosrun tf2_tools view_frames.py
evince frames.pdf

# TF 상태 확인
rosrun tf tf_monitor
```

### 지도 품질이 나쁜 경우

1. `max_range`를 실제 환경에 맞게 줄이기 (기본 8.0m)
2. `optimize_every_n_nodes`를 낮춰 루프 클로저를 더 자주 수행
3. 로봇 이동 속도를 줄여 스캔 오버랩 증가

---

## 파일 구조

```
woosh_slam_cartographer/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── cartographer.launch      # 메인 SLAM 스택
│   └── save_map.launch          # 지도 저장
├── config/
│   └── cartographer_2d.lua      # Cartographer 설정
├── rviz/
│   └── cartographer_debug.rviz  # RViz 시각화
└── docs/
    └── cartographer_guide.md    # 이 문서
```
