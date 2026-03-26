# Navigation 구현 체크리스트

**프로젝트**: gap_detect_v3_project
**작성일**: 2026-03-24
**참조**: [PLAN.md](PLAN.md)

---

## Phase 1: SDK Bridge 최소 구현

- [x] **P1-1** `woosh_sensor_bridge.py`: `/odom` → `/odom_raw` 토픽명 변경
- [x] **P1-2** `woosh_sensor_bridge.py`: `publish_tf` 파라미터 추가 (기본값 `true`)
- [x] **P1-3** `woosh_service_driver.py`: `/odom` 구독 경로를 EKF 결과 `/odom`으로 업데이트 확인
- [x] **P1-4** `sensor_bridge.launch` 생성 (단독 실행용)
- [x] **P1-5** 검증: `rostopic echo /odom_raw` 10Hz 이상 수신 확인
- [x] **P1-6** 검증: `rostopic echo /scan` 10Hz 이상 수신 확인
- [x] **P1-7** 검증: `rosrun tf tf_echo odom base_link` TF 수신 확인

## Phase 2: cmd_vel 연결

- [x] **P2-1** `woosh_bringup/scripts/cmd_vel_adapter.py` 신규 생성
  - [x] /cmd_vel 구독자 구현
  - [x] 속도 제한 클리핑 (max_linear: 0.12 m/s, max_angular: 0.5 rad/s)
  - [x] watchdog timer 구현 (1.0초 타임아웃 → 자동 정지)
  - [x] SDK twist_req() 호출
- [x] **P2-2** `cmd_vel_adapter.launch` 생성
- [ ] **P2-3** 검증: `rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.05}" -r 10` → 로봇 이동 확인
- [ ] **P2-4** 검증: /cmd_vel 토픽 중단 → 1초 내 로봇 정지 확인

## Phase 3: TF 정리

- [x] **P3-1** `static_tf.launch` 생성 (base_link→laser, z=0.25m)
- [x] **P3-2** `bringup.launch` 생성: sensor_bridge + cmd_vel_adapter + static_tf 통합
- [ ] **P3-3** 검증: `rosrun tf view_frames` → frames.pdf 확인
- [ ] **P3-4** 검증: odom→base_link→laser 체인 완성 확인

## Phase 4: robot_localization EKF

- [ ] **P4-1** `woosh_localization` 패키지 생성
  - [ ] `CMakeLists.txt` 작성
  - [ ] `package.xml` 작성 (robot_localization 의존성 추가)
- [ ] **P4-2** `config/ekf_no_imu.yaml` 작성
  - [ ] `odom0: /odom_raw` 설정
  - [ ] `two_d_mode: true` 설정
  - [ ] `odom0_differential: false` 설정
  - [ ] `publish_tf: true` 설정
  - [ ] process_noise_covariance 값 설정
- [ ] **P4-3** `launch/ekf_localization.launch` 작성
- [ ] **P4-4** `woosh_sensor_bridge.py`: `publish_tf` 를 `false`로 변경 (EKF와 충돌 방지)
- [ ] **P4-5** 검증: `rostopic echo /odom` 수신 확인
- [ ] **P4-6** 검증: TF(odom→base_link) 단일 발행 확인 (중복 없음)
- [ ] **P4-7** 검증: EKF 결과가 odom_raw보다 스무딩됨 확인 (rqt_plot 비교)

## Phase 5: RViz 검증

- [ ] **P5-1** `sensor_debug.rviz` 생성 (/scan, /odom, TF 표시)
- [ ] **P5-2** `sensor_debug.launch` 생성 (bringup + ekf + rviz)
- [ ] **P5-3** 검증: RViz에서 로봇 이동 궤적 확인
- [ ] **P5-4** 검증: LiDAR scan과 실제 환경 일치 확인
- [ ] **P5-5** 검증: base_link 기준 laser frame 방향 올바름 확인

## Phase 6: GMapping SLAM

- [ ] **P6-1** `gmapping.launch` 수정: `/odom` 구독 확인 (EKF 결과 사용)
- [ ] **P6-2** `gmapping_params.yaml` 튜닝 (필요 시)
- [ ] **P6-3** `slam_gmapping.launch` 생성 (bringup + ekf + gmapping + rviz)
- [ ] **P6-4** 검증: `roslaunch woosh_slam_gmapping slam_gmapping.launch` 기동 확인
- [ ] **P6-5** 검증: RViz에서 `/map` 증분 업데이트 확인
- [ ] **P6-6** 로봇 수동 조작으로 환경 탐색 (전체 맵 커버)
- [ ] **P6-7** 맵 저장: `roslaunch woosh_slam_gmapping save_map.launch map_name:=my_map`
- [ ] **P6-8** 저장된 `.pgm` + `.yaml` 파일 확인

## Phase 7: AMCL Localization

- [ ] **P7-1** `amcl.launch` 수정: `/odom` 구독 확인
- [ ] **P7-2** `amcl_params.yaml` 튜닝
  - [ ] `odom_model_type: diff` (differential drive 가정)
  - [ ] `min_particles` / `max_particles` 설정
  - [ ] `update_min_d` / `update_min_a` 설정
- [ ] **P7-3** `localization_amcl.launch` 생성 (bringup + ekf + map_server + amcl + rviz)
- [ ] **P7-4** 검증: `roslaunch woosh_slam_amcl localization_amcl.launch map_file:=...` 기동 확인
- [ ] **P7-5** 검증: RViz에서 `/initialpose` 설정 후 particle cloud 수렴 확인
- [ ] **P7-6** 검증: `rostopic echo /amcl_pose` 안정적 수신 확인
- [ ] **P7-7** 검증: TF(map→odom) 발행 확인

## Phase 8: move_base Navigation

- [x] **P8-1** `woosh_navigation_mb` 패키지 생성
  - [x] `CMakeLists.txt`, `package.xml` 작성
- [x] **P8-2** config 파일 작성
  - [x] `move_base_params.yaml` (controller_frequency: 5.0 Hz)
  - [x] `costmap_common_params.yaml` (woosh_costmap 공유)
  - [x] `global_costmap_params.yaml` (woosh_costmap 공유)
  - [x] `local_costmap_params.yaml`
  - [x] `global_planner_params.yaml` (navfn/Dijkstra)
  - [x] `local_planner_params.yaml` (dwa_local_planner)
- [x] **P8-3** `navigation.launch` 생성 (sensor_bridge + map_server + amcl + move_base + rviz)
- [x] **P8-4** `navigation.rviz` 생성 (static map, global/local costmap, path, particle cloud, TF)
- [ ] **P8-5** 검증: move_base 노드 기동 확인
- [ ] **P8-6** 검증: global/local costmap 표시 확인
- [ ] **P8-7** 검증: RViz 2D Nav Goal → 로봇 이동 확인
- [ ] **P8-8** 검증: `rostopic echo /move_base/result` SUCCESS 확인

## Phase 9: 튜닝 및 통합

- [ ] **P9-1** TR-200 실제 footprint 측정 → `costmap_common_params.yaml` 업데이트
- [ ] **P9-2** `inflation_radius` 튜닝 (협소 통로 테스트)
- [ ] **P9-3** local planner `max_vel_x` / `min_vel_x` 튜닝 (현재 max: 0.12 m/s)
- [ ] **P9-4** recovery behavior 테스트 (목표 근처 장애물 시)
- [ ] **P9-5** 3회 연속 navigation 성공 확인
- [ ] **P9-6** `main_system_operation.py`: navigation 통합 (필요 시)

## Phase 10: 선택 사항 (고도화)

- [ ] **OPT-1** Cartographer SLAM 테스트 및 GMapping 결과 비교
- [ ] **OPT-2** Cartographer pure localization 테스트
- [ ] **OPT-3** IMU 데이터 SDK 제공 여부 확인 → `imu_bridge.py` 구현
- [ ] **OPT-4** `ekf_with_imu.yaml` 적용 및 odom 품질 비교
- [ ] **OPT-5** TEB local planner 테스트 (좁은 공간)
- [ ] **OPT-6** `woosh_description` URDF 완성 (3D 모델)
- [ ] **OPT-7** dynamic reconfigure 적용 (costmap 실시간 튜닝)
- [ ] **OPT-8** multi-robot 통합 (Doosan a0912 + TR-200 동시 navigation)

---

## 빠른 참조: 주요 검증 명령어

```bash
# 토픽 확인
rostopic list
rostopic hz /odom_raw
rostopic hz /scan
rostopic hz /odom

# TF 확인
rosrun tf tf_echo odom base_link
rosrun tf tf_echo map odom
rosrun tf view_frames && evince frames.pdf

# 노드 그래프
rqt_graph

# 로그 확인
rqt_console
```

---

## 완료 기준 요약

| 단계 | 완료 기준 |
|------|----------|
| Phase 1 | /odom_raw + /scan 10Hz 이상 안정 발행 |
| Phase 2 | /cmd_vel 수신 → 로봇 이동, 타임아웃 → 정지 |
| Phase 3 | TF 트리 완성, 중복 없음 |
| Phase 4 | /odom EKF 발행, TF 단일화 |
| Phase 5 | RViz 시각화 정합성 확인 |
| Phase 6 | GMapping 맵 생성 및 저장 성공 |
| Phase 7 | AMCL particle 수렴, TF(map→odom) 안정 |
| Phase 8 | move_base 목표 이동 성공 |
| Phase 9 | 3회 연속 navigation 성공 |
