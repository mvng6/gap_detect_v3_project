-- ============================================================
-- cartographer_2d_localization.lua — Woosh TR-200 Pure Localization 설정
-- ============================================================
-- 사용 조건:
--   - SLAM으로 생성된 .pbstream 파일이 필요
--   - 새로운 서브맵을 생성하지 않고 기존 맵에서 위치만 추적
--   - 단일 2D LiDAR (/scan 토픽)
--   - 합성 오도메트리 (/odom 토픽, woosh_sensor_bridge 발행)
--   - IMU 없음
-- ============================================================

include "cartographer_2d.lua"

-- ============================================================
-- Pure Localization 전용 설정
-- ============================================================
-- 새 서브맵 생성을 중지하고 기존 맵에서 로컬라이제이션만 수행
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 로컬라이제이션 모드에서는 더 자주 최적화 수행 (위치 수렴 가속)
POSE_GRAPH.optimize_every_n_nodes = 20

-- 글로벌 제약 조건을 더 적극적으로 검색 (위치 재탐색 성능 향상)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

return options
