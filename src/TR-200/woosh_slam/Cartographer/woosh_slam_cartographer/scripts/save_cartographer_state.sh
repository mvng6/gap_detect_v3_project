#!/bin/bash
# ============================================================
# save_cartographer_state.sh
# Cartographer SLAM 상태를 .pbstream 파일로 저장
# ============================================================
# 사용법:
#   rosrun woosh_slam_cartographer save_cartographer_state.sh /path/to/output.pbstream
#
# 또는 launch 파일을 통해:
#   roslaunch woosh_slam_cartographer save_state.launch map_name:=my_map
# ============================================================

set -e

OUTPUT_FILE="${1:-/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream}"
OUTPUT_DIR="$(dirname "$OUTPUT_FILE")"

# 출력 디렉터리 생성
mkdir -p "$OUTPUT_DIR"

echo "============================================================"
echo " Cartographer 상태 저장"
echo " 출력 파일: $OUTPUT_FILE"
echo "============================================================"

# 1. 현재 트라젝토리 종료
echo "[1/2] 트라젝토리 종료 중..."
if rosservice call /finish_trajectory "trajectory_id: 0" 2>/dev/null; then
    echo "  트라젝토리 종료 완료"
else
    echo "  트라젝토리가 이미 종료되었거나 서비스를 찾을 수 없습니다 (계속 진행)"
fi

# 2. 상태 저장
echo "[2/2] .pbstream 파일 저장 중..."
rosservice call /write_state "filename: '${OUTPUT_FILE}'
include_unfinished_submaps: true"

echo "============================================================"
echo " 저장 완료: $OUTPUT_FILE"
echo ""
echo " Pure Localization으로 사용:"
echo "   roslaunch woosh_slam_cartographer cartographer_localization.launch \\"
echo "     state_file:=$OUTPUT_FILE"
echo "============================================================"
