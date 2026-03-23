# GMapping SLAM 가이드 — woosh_slam_gmapping

Woosh TR-200 모바일 로봇용 GMapping SLAM 패키지 사용 가이드입니다.

---

## 개요

GMapping은 **Rao-Blackwellized 파티클 필터** 기반의 온라인 SLAM 알고리즘으로,
사전 맵 없이 레이저 스캔과 오도메트리만으로 실시간 OccupancyGrid 지도를 생성합니다.

| 항목 | AMCL (`woosh_slam_amcl`) | GMapping (`woosh_slam_gmapping`) |
|------|--------------------------|----------------------------------|
| 목적 | 기존 맵에서 위치 추정 | 새 맵 생성 |
| 맵 필요 여부 | 필요 (사전 제작) | 불필요 |
| 출력 | `/amcl_pose`, `/particlecloud` | `/map` (실시간 업데이트) |
| TF 발행 | `map → odom` | `map → odom` |
| 파티클 수 | 500~3000 | 30 (맵 가설 포함 → 비용 높음) |
| 권장 순서 | GMapping으로 맵 생성 후 사용 | 최초 환경 탐색 시 사용 |

---

## TF 트리

```
map (고정 프레임)
 └── odom    ← slam_gmapping 발행 (map→odom TF)
      └── base_link  ← woosh_sensor_bridge 발행 (odom→base_link TF)
           └── laser  ← static_transform_publisher (base_link→laser, z=0.25m)
```

AMCL과 동일한 TF 트리 구조이므로, GMapping으로 생성한 맵을 AMCL에서 바로 사용할 수 있습니다.

---

## 노드 구성

### 1. `static_transform_publisher` (tf2_ros)
- 역할: 레이저 센서 위치 고정 TF 발행
- 변환: `base_link → laser` (z=0.25m 오프셋)
- AMCL `amcl.launch`와 동일한 오프셋

### 2. `woosh_sensor_bridge` (woosh_slam_amcl 패키지 스크립트 재사용)
- 역할: Woosh SDK WebSocket → ROS 토픽 변환
- 발행:
  - `/scan` (`sensor_msgs/LaserScan`) — 레이저 스캔 데이터
  - `/odom` (`nav_msgs/Odometry`) — twist 수치 적분 오도메트리
  - TF `odom → base_link`
- `robot_identity: gmapping_bridge` — AMCL의 `amcl_bridge`와 SDK 연결 구분

### 3. `slam_gmapping` (gmapping 패키지)
- 역할: `/scan` + `/odom` + TF 입력 → 지도 생성
- 발행:
  - `/map` (`nav_msgs/OccupancyGrid`) — 실시간 업데이트 점유 격자
  - TF `map → odom`
- 파라미터: `config/gmapping_params.yaml` 참조

---

## 빠른 시작

### 사전 준비

```bash
# 컨테이너 진입
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws
source devel/setup.bash

# 빌드 확인
catkin build woosh_slam_gmapping
source devel/setup.bash
```

### 1단계: GMapping 실행

```bash
# 기본 실행 (RViz 포함)
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2

# RViz 없이 실행
roslaunch woosh_slam_gmapping gmapping.launch robot_ip:=169.254.128.2 launch_rviz:=false

# woosh_service_driver와 통합 실행 (권장)
rosrun woosh_bringup woosh_service_driver.py slam
```

### 2단계: 환경 탐색

로봇을 수동으로 이동시키거나 `rosservice call /mobile_move`를 사용하여
지도를 생성할 영역 전체를 주행합니다.

지도 업데이트 확인:
```bash
rostopic hz /map     # 주기적 업데이트 확인 (보통 5~10초 간격)
rostopic echo /map/info  # 지도 크기 및 해상도 확인
```

RViz에서 `/map` 토픽이 점진적으로 채워지는 것을 확인합니다.

### 3단계: 지도 저장

GMapping이 실행 중인 상태에서 **별도 터미널**을 열어 실행합니다.

```bash
docker exec -it noetic_robot_system_ws bash
cd /root/catkin_ws && source devel/setup.bash

# 기본 저장 (woosh_map.pgm / woosh_map.yaml 덮어씀)
roslaunch woosh_slam_gmapping save_map.launch

# 새 이름으로 저장
roslaunch woosh_slam_gmapping save_map.launch map_name:=new_map

# 저장 경로 확인
ls -la /root/catkin_ws/src/TR-200/woosh_slam/maps/
```

저장 결과:
- `{map_name}.pgm` — 점유 격자 이미지 (흑: 장애물, 백: 자유 공간, 회: 미탐색)
- `{map_name}.yaml` — 해상도, 원점, 점유 임계값 메타데이터

### 4단계: 저장된 맵으로 AMCL 실행

```bash
rosrun woosh_bringup woosh_service_driver.py amcl \
  map_file:=/root/catkin_ws/src/TR-200/woosh_slam/maps/new_map.yaml
```

---

## 파라미터 레퍼런스

`config/gmapping_params.yaml` 주요 파라미터:

### 지도 설정
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `delta` | `0.03` | 지도 해상도 (m/cell). 기존 맵과 호환성 유지 |
| `xmin/xmax` | `-12.0/12.0` | 지도 X 범위 (m). 기존 환경(~16.6m)보다 작으므로 필요 시 확장 |
| `ymin/ymax` | `-12.0/12.0` | 지도 Y 범위 (m) |

### 파티클 필터
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `particles` | `30` | 파티클 수. 각 파티클이 전체 맵 가설 보유 → NUC에서 20~50 권장 |
| `resampleThreshold` | `0.5` | Neff/N < 0.5 시 재샘플링 |

### 업데이트 트리거
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `linearUpdate` | `0.1` | 이동 거리 임계값 (m). AMCL의 `update_min_d`에 대응 |
| `angularUpdate` | `0.2618` | 회전 각도 임계값 (rad, ~15°). AMCL의 `update_min_a`에 대응 |
| `temporalUpdate` | `-1.0` | 시간 기반 업데이트 비활성화 |

### 레이저 파라미터
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `maxRange` | `10.0` | 레이저 최대 물리 범위 (m) |
| `maxUrange` | `8.0` | 유효 사용 범위 (m). 원거리 노이즈 제거 |

### 스캔 매처
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `sigma` | `0.05` | 커널 표준편차 (m) |
| `kernelSize` | `1` | 탐색 커널 반경 (3×3) |
| `iterations` | `5` | ICP 반복 횟수 |
| `lstep` / `astep` | `0.05` | 경사 하강 스텝 크기 |

### TF 발행
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `transform_publish_period` | `0.05` | `map→odom` TF 발행 주기 (초, 20Hz) |

---

## 튜닝 가이드

### 지도 품질이 낮을 때

1. **`particles` 증가**: `30 → 50` (NUC 부하 확인 필요)
2. **`iterations` 증가**: `5 → 8` (스캔 매칭 정밀도 향상)
3. **`maxUrange` 감소**: `8.0 → 6.0` (노이즈 많은 원거리 측정값 제거)
4. **이동 속도 감소**: `woosh_service_driver.py`의 `max_speed` 감소

### CPU 과부하 시

1. **`particles` 감소**: `30 → 20`
2. **업데이트 트리거 증가**: `linearUpdate: 0.1 → 0.2`
3. **`iterations` 감소**: `5 → 3`
4. **`temporalUpdate: -1.0`** 유지 (비활성화 상태 확인)

### `/map`이 발행되지 않을 때

```bash
# sensor_bridge 상태 확인
rostopic hz /scan    # 10Hz 내외인지 확인
rostopic hz /odom    # 10Hz 내외인지 확인

# TF 트리 확인
rosrun tf2_tools view_frames.py   # frames.pdf 생성
rosrun tf tf_monitor              # TF 지연 모니터링

# GMapping 노드 상태 확인
rosnode info /slam_gmapping
```

---

## 트러블슈팅

### `woosh_sensor_bridge` 연결 실패
- 원인: Woosh SDK가 이미 최대 연결 수 초과 (`amcl_bridge`, `rviz_debug`, `twist_ctrl` 등이 동시 실행 중)
- 해결: 기존 연결 프로세스 종료 후 재시작

### `slam_gmapping`이 `scan` 토픽을 수신하지 못할 때
- 원인: `laser_frame`이 TF 트리에 없음
- 확인: `rosrun tf tf_echo base_link laser` 출력 확인
- 해결: `gmapping.launch`의 `laser_offset_*` 파라미터 점검

### 지도 원점이 로봇 시작 위치와 다를 때
- GMapping은 시작 시 로봇 위치를 원점(0, 0)으로 설정합니다.
- 로봇이 이미 이동한 상태에서 시작하면 맵 원점이 달라집니다.
- 권장: 로봇을 초기 위치에 놓고 GMapping 실행

### `map_saver`가 종료되지 않을 때
- 원인: `/map` 토픽이 아직 발행되지 않음
- 해결: `rostopic hz /map`으로 발행 확인 후 `save_map.launch` 실행

---

## 설계 결정 사항

### `woosh_sensor_bridge.py` 공유 재사용
GMapping과 AMCL은 동일한 센서 입력(`/scan`, `/odom`, TF)을 필요로 합니다.
`woosh_sensor_bridge.py`를 복사하지 않고 `pkg="woosh_slam_amcl"` 참조를 통해
단일 소스를 유지합니다. 향후 브릿지 코드 수정 시 AMCL과 GMapping 모두에 자동 반영됩니다.

### 지도 저장 경로 통합
`save_map.launch`의 기본 저장 경로를 `woosh_slam/maps/`로 지정하여
GMapping으로 생성한 맵을 파일 이동 없이 즉시 AMCL에서 사용할 수 있습니다.

### `robot_identity: gmapping_bridge`
Woosh SDK는 동일 IP에서 여러 WebSocket 클라이언트 연결을 허용하며,
`robot_identity` 문자열로 구분합니다. `gmapping_bridge`를 사용하여
`amcl_bridge`, `rviz_debug`, `twist_ctrl`과 충돌 없이 동시 실행이 가능합니다.
