-- ============================================================
-- cartographer_2d_localization_fix.lua — Woosh TR-200 Fixed-Map Localization
-- ============================================================
-- 사용 조건 (AMCL과 동일한 개념):
--   - SLAM으로 생성된 .pbstream 파일 로드
--   - 고정된 맵 위에서 로봇 위치만 추정
--   - /map 토픽은 로드한 원본 맵 그대로 유지 (cartographer_occupancy_grid_node 미실행)
--   - 단일 2D LiDAR (/scan), 합성 오도메트리 (/odom), IMU 없음
--
-- carto_loc_nonfix 와의 차이:
--   nonfix → cartographer_occupancy_grid_node 실행 → /map이 지속적으로 업데이트됨
--   fix    → cartographer_occupancy_grid_node 미실행 → /map 발행 없음 (위치 추정만 수행)
-- ============================================================

include "cartographer_2d.lua"

-- ============================================================
-- Pure Localization 기반 (서브맵 트리머 활성화)
-- max_submaps_to_keep >= 2 필수 (Cartographer 내부 제약)
-- ============================================================
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 로컬라이제이션 성능 최적화
POSE_GRAPH.optimize_every_n_nodes = 20

-- 글로벌 제약 검색 강도 향상 (초기 위치 수렴 가속)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

return options
