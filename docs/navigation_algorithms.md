# Navigation 알고리즘 개요 — Woosh TR-200

**작성일**: 2026-03-26
**대상 로봇**: Woosh TR-200 (차동 구동, 최대 선속도 0.12 m/s)
**ROS 버전**: ROS1 Noetic

---

## 전체 구조

```
[지도 생성] SLAM
     ↓ (map 파일 저장)
[위치 추정] Localization  →  TF: map → odom
     ↓
[경로 계획] Global Planner  →  전역 경로 (global path)
     ↓
[장애물 회피] Local Planner  →  /cmd_vel
     ↓
[속도 전달] cmd_vel_adapter  →  Woosh SDK (WebSocket)
```

---

## 1. SLAM — 지도 생성

두 가지 SLAM 알고리즘이 구현되어 있으며, 운영 환경에 따라 선택 사용한다.

### 1-A. GMapping (기본 운용)

| 항목 | 값 |
|------|----|
| 알고리즘 | Rao-Blackwellized Particle Filter SLAM |
| 패키지 | `woosh_slam_gmapping` |
| 파티클 수 | 30개 (NUC 성능 기준) |
| 지도 해상도 | 0.03 m/cell |
| 스캔 매처 | ICP 기반, 반복 5회 |
| 업데이트 트리거 | 10 cm 이동 / 15° 회전 |
| LiDAR 유효 범위 | 8.0 m (물리 최대 10.0 m) |
| 출력 | `/map` (OccupancyGrid), TF `map → odom` |

**핵심 특징**: 각 파티클이 전체 지도 가설을 독립적으로 유지한다. AMCL(500~3000개)보다 훨씬 적은 파티클로 동작하지만, 파티클당 메모리·연산 비용이 높다.

**오도메트리 노이즈 모델** (합성 오도메트리이므로 보수적 설정):

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `srr` | 0.1 | 이동 → 이동 노이즈 비율 |
| `srt` | 0.2 | 이동 → 회전 노이즈 비율 |
| `str` | 0.1 | 회전 → 이동 노이즈 비율 |
| `stt` | 0.2 | 회전 → 회전 노이즈 비율 |

### 1-B. Cartographer (고정밀 대안)

| 항목 | 값 |
|------|----|
| 알고리즘 | Graph SLAM (Ceres Solver 기반 Pose Graph 최적화) |
| 패키지 | `woosh_slam_cartographer` |
| 출력 | `/map` or `/carto_map`, TF `map → odom`, `.pbstream` 상태 파일 |
| 특이사항 | Localization 전용 모드(`carto_loc_fix`, `carto_loc_nonfix`) 지원 |

---

## 2. Localization — 위치 추정

저장된 맵 위에서 로봇의 현재 위치를 실시간으로 추정한다.
**AMCL** 또는 **Cartographer Localization** 중 하나를 선택 사용한다.

### 2-A. AMCL (기본 운용)

**Adaptive Monte Carlo Localization** — 파티클 필터 기반 위치 추정.

| 항목 | 값 |
|------|----|
| 패키지 | `woosh_slam_amcl` |
| 파티클 수 | 500 ~ 3,000개 (KLD 적응 샘플링) |
| 레이저 모델 | `likelihood_field` (실내 환경에서 beam 모델보다 강건) |
| 레이저 서브샘플링 | 60 빔/스캔 (전 빔 사용 시 CPU 과부하 방지) |
| 오도메트리 모델 | `diff` (차동 구동) |
| 업데이트 트리거 | 5 cm 이동 / 10° 회전 |
| 리샘플링 간격 | 2회 필터 업데이트마다 1회 |
| `transform_tolerance` | 0.5 s (WebSocket 통신 지연 대응) |
| 출력 | `/amcl_pose`, TF `map → odom` |

**오도메트리 노이즈 파라미터** (보수적 설정 — 합성 오도메트리 특성 반영):

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `odom_alpha1` | 0.3 | 회전 → 회전 노이즈 |
| `odom_alpha2` | 0.3 | 이동 → 회전 노이즈 |
| `odom_alpha3` | 0.2 | 이동 → 이동 노이즈 |
| `odom_alpha4` | 0.2 | 회전 → 이동 노이즈 |

> **오도메트리 특이사항**: TR-200은 바퀴 엔코더를 직접 제공하지 않는다.
> `woosh_sensor_bridge.py`가 SDK의 `PoseSpeed.twist`(linear, angular)를 dt 적분하여 합성 오도메트리를 생성한다. 이 때문에 알파 값이 일반적인 엔코더 기반 로봇보다 높게 설정되어 있다.

### 2-B. Cartographer Localization

| 모드 | 설명 |
|------|------|
| `carto_loc_fix` | 고정 맵 — 서브맵 갱신 없음 (AMCL 유사) |
| `carto_loc_nonfix` | 서브맵 업데이트 포함 — 환경 변화 적응 가능 |

`.pbstream` 상태 파일 필요. Cartographer Pose Graph 기반으로 scan matching을 수행한다.

---

## 3. Costmap — 비용 지도

경로 계획과 장애물 회피를 위한 격자 비용 지도. Global / Local 두 레이어로 구성된다.

### 공통 설정

| 항목 | 값 |
|------|----|
| 로봇 Footprint | 직사각형 폴리곤, 0.65 m × 0.45 m |
| Footprint 패딩 | 0.02 m |
| 센서 소스 | `/scan` (2D LiDAR, `laser` 프레임) |
| `transform_tolerance` | 0.5 s |

### 3-A. Global Costmap

| 항목 | 값 |
|------|----|
| 기준 프레임 | `map` |
| 해상도 | 0.05 m/cell |
| 업데이트 주기 | 2.0 Hz |
| 장애물 마킹 범위 | 6.0 m |
| 레이트레이싱 범위 | 8.0 m |

**레이어 구성 (순서대로 적용)**:

```
StaticLayer   → map_server의 /map 구독 (lethal_cost_threshold: 65)
ObstacleLayer → LiDAR 기반 동적 장애물 마킹 (combination: Maximum)
InflationLayer → inflation_radius: 0.55 m, cost_scaling_factor: 3.0
```

**Inflation 반경 설계 근거**:
- 외접 반경: sqrt(0.325² + 0.225²) = 0.395 m
- 안전 마진 0.155 m = 오도메트리 오차 ~5 cm + WebSocket 지연 보정 ~5 cm + 여유 ~5.5 cm
- 합산: **0.55 m**

### 3-B. Local Costmap

| 항목 | 값 |
|------|----|
| 기준 프레임 | `odom` (Rolling Window) |
| 크기 | 3.0 m × 3.0 m |
| 해상도 | 0.05 m/cell |
| 업데이트 주기 | 5.0 Hz (controller_frequency와 동일) |
| 장애물 마킹 범위 | 3.0 m (Global 6.0 m보다 좁게 — 근거리 집중) |

**레이어 구성**:

```
ObstacleLayer → LiDAR /scan 기반 동적 장애물
InflationLayer → inflation_radius: 0.55 m, cost_scaling_factor: 3.0
```

Static Layer 불필요 — Rolling Window이므로 지도 파일을 사용하지 않는다.

---

## 4. Global Planner — 전역 경로 계획

| 항목 | 값 |
|------|----|
| 알고리즘 | **Dijkstra** (`navfn/NavfnROS`) |
| `use_dijkstra` | `true` (A* 대신 사용 — 소규모 맵에서 안정적) |
| `allow_unknown` | `true` (미지 영역 통과 허용) |
| 탐색 범위 | 전체 맵 사용 (`planner_window: 0.0`) |
| 실행 주기 | 1.0 Hz |
| 최대 탐색 대기 | 5.0 초 |

**Dijkstra vs A*** 선택 이유: 소규모 실내 맵(~12 m × 12 m)에서 두 알고리즘의 속도 차이는 미미하며, Dijkstra가 비용 전파에서 더 일관된 결과를 제공한다.

---

## 5. Local Planner — 지역 장애물 회피

| 항목 | 값 |
|------|----|
| 알고리즘 | **DWA (Dynamic Window Approach)** (`dwa_local_planner/DWAPlannerROS`) |
| 실행 주기 | 5.0 Hz |

### 속도 한계

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `max_vel_x` | 0.10 m/s | 로봇 최대(0.12) 대비 약간 낮게 — 실내 안전 |
| `min_vel_x` | 0.01 m/s | 너무 낮으면 정지 판단 지연 |
| `max_vel_theta` | 0.5 rad/s | `cmd_vel_adapter` 클리핑 값과 동일 |
| `min_vel_theta` | 0.05 rad/s | 너무 낮으면 회전 정체 |

### 가속도 한계 (WebSocket 100 ms 지연 대응 — 보수적 설정)

| 파라미터 | 값 |
|----------|----|
| `acc_lim_x` | 0.3 m/s² |
| `acc_lim_theta` | 0.5 rad/s² |

### DWA 시뮬레이션 파라미터

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `sim_time` | 2.0 s | 예측 시간 — 짧으면 근시안적, 길면 계산 부하 |
| `vx_samples` | 6 | 선속도 샘플 수 |
| `vtheta_samples` | 20 | 각속도 샘플 수 |

### 목표 허용 오차

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `xy_goal_tolerance` | 0.15 m | 합성 오도메트리 정확도 고려 |
| `yaw_goal_tolerance` | 0.15 rad | ≈ 8.6° |

### 비용 가중치

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `path_distance_bias` | 32.0 | 전역 경로 추종 강도 |
| `goal_distance_bias` | 24.0 | 목표 접근 강도 |
| `occdist_scale` | 0.02 | 장애물 회피 강도 |

---

## 6. cmd_vel_adapter — 속도 명령 브릿지

move_base가 발행하는 `/cmd_vel`을 Woosh SDK `twist_req()`로 변환하는 중간 계층.

| 항목 | 값 |
|------|----|
| 구독 토픽 | `/cmd_vel` |
| 최대 선속도 클리핑 | 0.12 m/s |
| 최대 각속도 클리핑 | 0.5 rad/s |
| Watchdog 타임아웃 | 1.0 s (토픽 중단 시 자동 정지) |

---

## 7. 실행 조합 요약

| 목적 | 실행 명령 |
|------|-----------|
| SLAM (지도 생성) | `rosrun woosh_bringup woosh_service_driver.py gmap` |
| AMCL만 (위치 추정) | `rosrun woosh_bringup woosh_service_driver.py amcl map_file:=...` |
| AMCL + Global Costmap | `rosrun woosh_bringup woosh_service_driver.py amcl nav_on map_file:=...` |
| **AMCL + move_base (자율주행)** | `rosrun woosh_bringup woosh_service_driver.py amcl move_base_on map_file:=...` |
| Cartographer(fixed) + move_base | `rosrun woosh_bringup woosh_service_driver.py carto_loc_fix move_base_on state_file:=...` |

---

## 8. TR-200 특수 고려 사항

| 제약 | 대응 |
|------|------|
| WebSocket 통신 지연 ~100 ms | `controller_frequency: 5 Hz`, `transform_tolerance: 0.5 s`, 보수적 가속도 |
| 바퀴 엔코더 미제공 | SDK `PoseSpeed.twist` dt 적분으로 합성 오도메트리 생성 — 오도메트리 노이즈 알파 값 높게 설정 |
| IMU 미제공 | EKF 융합 미적용 (외부 IMU 추가 시 재검토) |
| 차동 구동 | `odom_model_type: diff`, `max_vel_y: 0.0`, `vy_samples: 1` |
