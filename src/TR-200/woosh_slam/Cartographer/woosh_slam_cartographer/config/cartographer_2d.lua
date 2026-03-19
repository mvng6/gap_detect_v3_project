-- ============================================================
-- cartographer_2d.lua — Woosh TR-200 Cartographer 2D SLAM 설정
-- ============================================================
-- 사용 조건:
--   - 단일 2D LiDAR (/scan 토픽)
--   - 합성 오도메트리 (/odom 토픽, woosh_sensor_bridge 발행)
--   - IMU 없음
--   - 실내 환경 (복도, 창고 등)
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 고정 맵 프레임
  map_frame = "map",

  -- Cartographer가 추적하는 프레임 (로봇 베이스)
  tracking_frame = "base_link",

  -- Cartographer가 map에 붙일 프레임 (map→odom TF를 발행)
  -- woosh_sensor_bridge가 odom→base_link를 발행하므로 odom을 지정
  published_frame = "odom",

  -- 오도메트리 좌표계 이름
  odom_frame = "odom",

  -- false: woosh_sensor_bridge가 odom→base_link TF를 발행하므로 Cartographer가 odom을 제공하지 않음
  provide_odom_frame = false,

  -- 2D 환경이므로 XY 평면에 투영
  publish_frame_projected_to_2d = true,

  -- woosh_sensor_bridge가 발행하는 /odom 토픽 사용
  use_odometry = true,

  -- 사용하지 않는 센서
  use_nav_sat = false,
  use_landmarks = false,

  -- 단일 2D LiDAR (/scan 토픽)
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- IMU 없음
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- ============================================================
-- MapBuilder 설정
-- ============================================================
MAP_BUILDER.use_trajectory_builder_2d = true
-- 3D 비활성화 (메모리/CPU 절약)
MAP_BUILDER.num_background_threads = 2

-- ============================================================
-- 2D TrajectoryBuilder 설정
-- ============================================================
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 레이저 유효 범위 설정 (GMapping의 maxUrange=8.0과 동일)
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- max_range를 초과하는 측정값을 처리할 가상 거리
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

-- 실시간 스캔 매칭 (CSM: Correlative Scan Matching)
-- WebSocket 기반의 불규칙한 오도메트리 보정에 효과적
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 스캔 매처 (Ceres 기반)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.0

-- 적응형 voxel 필터 (포인트 수 제어)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0

-- 오도메트리 가중치 (합성 오도메트리이므로 다소 낮게 설정)
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- ============================================================
-- PoseGraph (루프 클로저) 설정
-- ============================================================
-- 90개 노드마다 전역 최적화 (NUC 성능 고려)
POSE_GRAPH.optimize_every_n_nodes = 90

-- 루프 클로저 제약 조건 검색 범위
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- 최적화 반복 횟수
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.max_num_final_iterations = 200

return options
