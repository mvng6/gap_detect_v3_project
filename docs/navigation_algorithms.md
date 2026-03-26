# Navigation 알고리즘 개요 — Woosh TR-200

**작성일**: 2026-03-26
**최종 수정**: 2026-03-26 (#001~#008 버그픽스 반영)
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
[속도 전달] SmoothTwistController (cmd_vel passthrough)  →  Woosh SDK (WebSocket)
```

> **구조 변경 이력 (#002)**: 기존에는 `cmd_vel_adapter`(서브프로세스)가 별도 WebSocket 연결을 열어 `/cmd_vel`을 로봇에 전달하였으나, 이중 WebSocket 연결로 명령 충돌이 발생하였다. 현재는 `SmoothTwistController` 내부의 **cmd_vel passthrough** 기능이 기존 WebSocket 연결을 재사용하여 `/cmd_vel`을 전달한다.

---

## 1. SLAM — 지도 생성

두 가지 SLAM 알고리즘이 구현되어 있으며, 운영 환경에 따라 선택 사용한다.

### 1-A. GMapping (기본 운용)

| 항목 | 값 |
|------|-----|
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
|------|-----|
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
|------|-----|
| 패키지 | `woosh_slam_amcl` |
| 파티클 수 | 500 ~ 3,000개 (KLD 적응 샘플링) |
| 레이저 모델 | `likelihood_field` (실내 환경에서 beam 모델보다 강건) |
| 레이저 서브샘플링 | 60 빔/스캔 (전 빔 사용 시 CPU 과부하 방지) |
| 오도메트리 모델 | `diff` (차동 구동) |
| 업데이트 트리거 | 5 cm 이동 / 10° 회전 (`update_min_a: 0.1745 rad`) |
| 리샘플링 간격 | 2회 필터 업데이트마다 1회 |
| `transform_tolerance` | 0.5 s (WebSocket 통신 지연 대응) |
| 맵 취득 방식 | `use_map_topic: true` — `/map` 토픽 구독 (map_server 준비 완료 후 수신) |
| 출력 | `/amcl_pose`, TF `map → odom` |

**오도메트리 노이즈 파라미터** (보수적 설정 — 합성 오도메트리 특성 반영):

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `odom_alpha1` | 0.3 | 회전 → 회전 노이즈 |
| `odom_alpha2` | 0.3 | 이동 → 회전 노이즈 |
| `odom_alpha3` | 0.2 | 이동 → 이동 노이즈 |
| `odom_alpha4` | 0.2 | 회전 → 이동 노이즈 |

**초기 포즈 불확실도**:

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `initial_cov_xx` | 0.25 | x 방향 0.5 m 표준편차 |
| `initial_cov_yy` | 0.25 | y 방향 0.5 m 표준편차 |
| `initial_cov_aa` | 0.0685 | 각도 약 15° 표준편차 |

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

### 공통 설정 (`costmap_common_params.yaml`)

| 항목 | 값 |
|------|-----|
| 로봇 Footprint | 직사각형 폴리곤, 0.65 m × 0.45 m |
| Footprint 패딩 | 0.02 m |
| 센서 소스 | `/scan` (2D LiDAR, `laser` 프레임) |
| `transform_tolerance` | 0.5 s |
| `inf_is_valid` | false (LiDAR inf 반환값 무효 처리) |

### 3-A. Global Costmap (`woosh_costmap` 패키지)

| 항목 | 값 |
|------|-----|
| 기준 프레임 | `map` |
| 해상도 | 0.05 m/cell (맵 원본 0.03 m보다 낮게 — NUC 성능 절약) |
| 업데이트 주기 | 2.0 Hz |
| 발행 주기 | 1.0 Hz (RViz용) |
| `always_send_full_costmap` | true (RViz 누락 방지) |
| 장애물 마킹 범위 | 6.0 m |
| 레이트레이싱 범위 | 8.0 m |

**레이어 구성 (순서대로 적용)**:

```
StaticLayer   → /map 구독, lethal_cost_threshold: 65, trinary_costmap: true
ObstacleLayer → LiDAR 기반 동적 장애물 마킹 (combination_method: Maximum)
InflationLayer → inflation_radius: 0.55 m, cost_scaling_factor: 3.0
```

**Static Layer 상세**:
- `track_unknown_space: true` — 미지 영역 유지 (NO_INFORMATION)
- `lethal_cost_threshold: 65` — map_server `occupied_thresh: 0.65`와 일치하여 벽 반영 안정화
- `subscribe_to_updates: true` — map_server 재발행 시 자동 반영

**Inflation 반경 설계 근거**:
- 내접 반경: 0.45 / 2 = 0.225 m
- 외접 반경: sqrt(0.325² + 0.225²) = 0.395 m
- 안전 마진 0.155 m = 합성 오도메트리 오차 ~5 cm + WebSocket 지연 보정 ~5 cm + 여유 ~5.5 cm
- 합산: **0.55 m**

### 3-B. Local Costmap (`woosh_navigation_mb` 패키지)

| 항목 | 값 |
|------|-----|
| 기준 프레임 | `odom` (Rolling Window) |
| 크기 | 3.0 m × 3.0 m |
| 해상도 | 0.05 m/cell |
| 업데이트 주기 | 5.0 Hz (controller_frequency와 동일) |
| 발행 주기 | 2.0 Hz (RViz용) |
| 장애물 마킹 범위 | 3.0 m (근거리 집중) |
| 레이트레이싱 범위 | **4.0 m** (Global 8.0 m보다 좁게 — 로컬 창 내 정확성 우선) |

**레이어 구성**:

```
ObstacleLayer → LiDAR /scan 기반 동적 장애물
InflationLayer → inflation_radius: 0.55 m, cost_scaling_factor: 3.0
```

Static Layer 불필요 — Rolling Window이므로 지도 파일을 사용하지 않는다.

> **크기 설계 근거**: DWA `sim_time(2.0s) × 최대속도(0.12 m/s) = 0.24 m` → 3.0 m는 충분한 여유.

---

## 4. Global Planner — 전역 경로 계획

| 항목 | 값 |
|------|-----|
| 알고리즘 | **Dijkstra** (`navfn/NavfnROS`) |
| `use_dijkstra` | `true` (A* 대신 사용 — 소규모 맵에서 안정적) |
| `allow_unknown` | `true` (미지 영역 통과 허용) |
| 탐색 범위 | 전체 맵 사용 (`planner_window_x/y: 0.0`) |
| `default_tolerance` | 0.0 (목표 비용 허용 없음 — `xy_goal_tolerance`에 위임) |
| 실행 주기 | 1.0 Hz |
| 최대 탐색 대기 | 5.0 초 |

**Dijkstra vs A*** 선택 이유: 소규모 실내 맵(~12 m × 12 m)에서 두 알고리즘의 속도 차이는 미미하며, Dijkstra가 비용 전파에서 더 일관된 결과를 제공한다.

---

## 5. Local Planner — 지역 장애물 회피

| 항목 | 값 |
|------|-----|
| 알고리즘 | **DWA (Dynamic Window Approach)** (`dwa_local_planner/DWAPlannerROS`) |
| 실행 주기 | 5.0 Hz |

### 속도 한계

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `max_vel_x` | 0.10 m/s | 로봇 최대(0.12) 대비 낮게 — 실내 안전 |
| `min_vel_x` | 0.01 m/s | 너무 낮으면 정지 판단 지연 |
| `max_vel_trans` | 0.10 m/s | 합성 병진 속도 한계 |
| `max_vel_theta` | 0.5 rad/s | cmd_vel_adapter 클리핑 값과 동일 |
| `min_vel_theta` | **0.01 rad/s** | 미세 각도 보정 허용 (0.05→0.01: 불감대 제거, #006) |
| `max_vel_y` | 0.0 m/s | 차동 구동 — 측면 이동 불가 |

### 가속도 한계

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `acc_lim_x` | 0.3 m/s² | WebSocket 100 ms 지연 대응 — 보수적 설정 |
| `acc_lim_theta` | **0.5 rad/s²** | DWA 탐색 공간 확보 필수 값 |

> **acc_lim_theta 설계 원칙**: DWA는 매 계획 주기(`sim_period = 1/5Hz = 0.2 s`)마다 현재 속도 ± (`acc_lim_theta × sim_period`) 범위만 샘플링한다. `acc_lim_theta = 0.5`이면 ±0.10 rad/s 샘플링 가능. `acc_lim_theta = 0.15`로 낮추면 ±0.03 rad/s만 샘플링되어 방향 전환이 필요한 모든 trajectory가 stop(0,0)보다 비용이 높아지고 DWA가 영구 stop을 출력한다 — 사용 금지 (#007 회귀 버그 교훈).

### DWA 시뮬레이션 파라미터

| 파라미터 | 값 | 설명 |
|----------|----|------|
| `sim_time` | 2.0 s | 예측 시간 — 짧으면 근시안적, 길면 계산 부하 |
| `sim_granularity` | 0.025 m | 경로 샘플링 해상도 |
| `angular_sim_granularity` | 0.025 rad | 각도 샘플링 해상도 |
| `vx_samples` | 6 | 선속도 샘플 수 |
| `vy_samples` | 1 | y 샘플 (차동 구동: 1개 고정) |
| `vtheta_samples` | 20 | 각속도 샘플 수 |
| `forward_point_distance` | 0.325 m | 전방 참조점 (TR-200 길이/2) |

### 목표 허용 오차

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `xy_goal_tolerance` | 0.15 m | 합성 오도메트리 정확도 고려 |
| `yaw_goal_tolerance` | 0.15 rad | ≈ 8.6° |
| `latch_xy_goal_tolerance` | false | xy 도달 후 방향도 함께 맞춤 |

### 비용 가중치

| 파라미터 | 현재값 | 변경 이력 | 의미 |
|----------|--------|-----------|------|
| `path_distance_bias` | **14.0** | 32.0 → 22.0 → 14.0 | 전역 경로 추종 강도 |
| `goal_distance_bias` | **44.0** | 24.0 → 36.0 → 44.0 | 목표 접근 강도 |
| `occdist_scale` | **0.05** | 0.02 → 0.05 | 장애물 회피 강도 |

**`path_distance_bias` 단계적 완화 배경**:
- 초기 32.0: AMCL/Carto localization 노이즈(수 cm)와 결합 시 DWA가 경로 좌우 미세 오차에도 max angular 포화 출력. 전진 중 angular = −0.1 rad/s 지배적, 선속도 최대 0.05 m/s 수준 정체, 이후 DWA oscillation recovery 발동 (#005~#006).
- 22.0으로 1차 완화: 전진 중 angular stdev = 0.065, 부호 반전 7.9% 잔존 → L/R 미세 진동 지속, 각도 오차 누적 → DWA 제자리 CCW 회전 돌입 → clearing_rotation 2회 (#008).
- 14.0으로 2차 완화: `goal_distance_bias`를 44.0으로 상향해 DWA가 목표 방향으로 적극 회전 선택하도록 유도 (path_cost < goal_cost → 빠른 각속도 선택).

### 정지 판단

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `stop_time_buffer` | 0.2 s | 장애물 전 정지 여유 시간 |
| `oscillation_reset_dist` | **0.1 m** | 진동 카운터 초기화 이동 거리 (0.05→0.1: 합성 오도메트리 부정확으로 인한 가짜 oscillation 감지 방지, #006) |

---

## 6. move_base 복구 동작

| 파라미터 | 값 | 비고 |
|----------|----|------|
| `controller_frequency` | 5.0 Hz | WebSocket 왕복 지연(~100ms) 고려 |
| `planner_frequency` | 1.0 Hz | 전역 경로는 낮은 빈도로 충분 |
| `planner_patience` | 5.0 s | 전역 경로 탐색 최대 대기 |
| `recovery_behavior_enabled` | true | |
| `clearing_rotation_allowed` | true | 제자리 회전 복구 허용 |
| `conservative_reset_dist` | 3.0 m | 이 거리 내 장애물만 보수적 초기화 |
| `oscillation_timeout` | **30.0 s** | 진동 감지 타임아웃 (10.0→30.0, #008) |
| `oscillation_distance` | **0.3 m** | 진동 카운터 초기화 이동 거리 (0.2→0.3, #008) |
| `shutdown_costmaps` | false | 목표 도달 후 costmap 유지 (재탐색 대비) |

**`oscillation_timeout` 연장 배경 (#008)**: DWA 제자리 회전 속도가 `acc_lim_theta(0.5) × sim_period(0.2s) = 0.1 rad/s`로 고착될 때, 10s 타임아웃으로는 약 57° 회전 후 clearing_rotation이 2회 발동되었다. 30s로 연장하면 최대 172° 커버 → 거의 모든 방향 전환 대응 가능.

---

## 7. cmd_vel_adapter — 속도 명령 브릿지

move_base가 발행하는 `/cmd_vel`을 Woosh SDK `twist_req()`로 전달하는 중간 계층.

**현재 구조 (변경 후, #002~#003)**:

| 항목 | 값 |
|------|-----|
| 구현 위치 | `SmoothTwistController` 내부 cmd_vel passthrough |
| WebSocket 연결 | 기존 연결 재사용 (별도 연결 없음) |
| 구독 토픽 | `/cmd_vel` |
| 최대 선속도 클리핑 | 0.12 m/s |
| 최대 각속도 클리핑 | 0.5 rad/s |
| Watchdog 타임아웃 | 1.0 s (토픽 중단 시 자동 정지) |
| twist 전송 방식 | fire-and-forget 비동기 태스크 (#003: WebSocket RTT 블로킹 방지) |

> **변경 배경 (#002)**: 기존 `cmd_vel_adapter.py`는 별도 서브프로세스로 두 번째 WebSocket 연결을 열었다. Woosh SDK가 중복 연결을 처리하는 과정에서 명령 드롭 및 연결 재설정이 발생하여 stop-and-go 현상의 주원인이 되었다. `SmoothTwistController`의 기존 WebSocket 연결을 재사용하는 passthrough 방식으로 전환하여 해결.

---

## 8. Navigation 로깅

자율주행 중 명령 속도를 실시간으로 CSV 파일에 기록한다 (#004).

| 항목 | 내용 |
|------|------|
| 파일 위치 | `src/TR-200/woosh_bringup/logs/nav_cmd_YYYYMMDD_HHMMSS.csv` |
| 컬럼 | `timestamp`, `elapsed_sec`, `source`, `linear_m_s`, `angular_rad_s` |
| `source` 값 | `"quintic"` (직선 이동) / `"cmd_vel"` (move_base 패스스루) |
| 용도 | DWA 파라미터 튜닝 — angular 포화, oscillation 패턴 분석 |

---

## 9. 실행 조합 요약

| 목적 | 실행 명령 |
|------|-----------|
| SLAM (지도 생성) | `rosrun woosh_bringup woosh_service_driver.py gmap` |
| AMCL만 (위치 추정) | `rosrun woosh_bringup woosh_service_driver.py amcl map_file:=...` |
| AMCL + Global Costmap | `rosrun woosh_bringup woosh_service_driver.py amcl nav_on map_file:=...` |
| **AMCL + move_base (자율주행)** | `rosrun woosh_bringup woosh_service_driver.py amcl move_base_on map_file:=...` |
| Cartographer(fixed) + move_base | `rosrun woosh_bringup woosh_service_driver.py carto_loc_fix move_base_on state_file:=...` |

---

## 10. TR-200 특수 고려 사항

| 제약 | 대응 |
|------|------|
| WebSocket 통신 지연 ~100 ms | `controller_frequency: 5 Hz`, `transform_tolerance: 0.5 s`, 보수적 가속도 (`acc_lim_x: 0.3 m/s²`) |
| 바퀴 엔코더 미제공 | SDK `PoseSpeed.twist` dt 적분으로 합성 오도메트리 생성 — 오도메트리 노이즈 알파 값 높게 설정, `oscillation_reset_dist` 상향 |
| WebSocket RTT 블로킹 | `twist_req()` fire-and-forget 비동기 태스크로 분리 (#003) — 실제 제어 주기 보장 |
| 이중 WebSocket 연결 금지 | cmd_vel passthrough를 SmoothTwistController에 통합 (#002) |
| IMU 미제공 | EKF 융합 미적용 (외부 IMU 추가 시 재검토) |
| 차동 구동 | `odom_model_type: diff`, `max_vel_y: 0.0`, `vy_samples: 1` |
| Localization 노이즈 → DWA angular 포화 | `path_distance_bias: 14.0` (완화), `goal_distance_bias: 44.0` (상향), `acc_lim_theta: 0.5` (탐색 공간 확보) |
