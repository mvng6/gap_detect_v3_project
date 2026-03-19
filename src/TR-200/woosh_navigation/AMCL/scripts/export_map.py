#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Woosh 맵 내보내기 유틸리티

Woosh 로봇 SDK에서 현재 로드된 맵을 가져와 ROS map_server 포맷(.pgm + .yaml)으로
저장합니다. 저장된 파일은 amcl.launch의 map_file 인자로 사용됩니다.

Example:
  rosrun woosh_slam_amcl export_map.py \\
    _robot_ip:=169.254.128.2 \\
    _output_dir:=/root/catkin_ws/src/TR-200/woosh_navigation/maps \\
    _map_name:=my_map
"""

import asyncio
import io
import logging
import os
import sys

import rospy
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WOOSH_ROBOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../woosh_robot_py"))
if WOOSH_ROBOT_DIR not in sys.path:
    sys.path.insert(0, WOOSH_ROBOT_DIR)

from woosh_robot import WooshRobot  # noqa: E402
from woosh_interface import CommuSettings, NO_PRINT  # noqa: E402
from woosh.proto.map.map_pack_pb2 import SceneData  # noqa: E402
from woosh.proto.robot.robot_pb2 import RobotInfo  # noqa: E402


def _open_rgba_image(png_bytes):
    if not png_bytes:
        return None
    try:
        img = Image.open(io.BytesIO(png_bytes))
        return img.convert("RGBA")
    except Exception as exc:
        rospy.logerr("PNG 이미지 열기 실패: %s", exc)
        return None


def _save_pgm_and_yaml(map_msg, output_dir, map_name):
    """
    SceneData.Map을 .pgm + .yaml 파일로 저장합니다.

    좌표계 변환:
      Woosh PNG: PIL row 0 = 이미지 상단 = 맵의 최대 y 방향
      map_server PGM: 파일의 첫 번째 행 = 맵의 하단 (origin.y)
      → PNG를 수직으로 뒤집어(flip) 저장해야 좌표계가 일치합니다.
    """
    rgba = _open_rgba_image(map_msg.map_png)
    if rgba is None:
        rospy.logerr("map_png가 없거나 유효하지 않습니다.")
        return False

    width, height = rgba.size
    rgba_pixels = rgba.load()

    # 수직 뒤집기 후 그레이스케일 변환
    # - 뒤집기: PIL row y → 저장 시 row (height-1-y)
    # - 투명 픽셀(alpha < 10) → 128 (ROS unknown 값)
    # - map_server: p(occ) = 1 - pixel/255
    #     pixel > 204 → free (p < 0.196)
    #     pixel < 89  → occupied (p > 0.65)
    #     128~204     → unknown
    gray_data = []
    for y in range(height - 1, -1, -1):  # 하단 행부터 저장
        row = []
        for x in range(width):
            r, g, b, a = rgba_pixels[x, y]
            if a < 10:
                row.append(128)  # unknown
            else:
                intensity = int(round((float(r) + float(g) + float(b)) / 3.0))
                row.append(intensity)
        gray_data.extend(row)

    # PGM 파일 저장 (binary PGM, P5 포맷)
    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    with open(pgm_path, "wb") as f:
        header = f"P5\n{width} {height}\n255\n"
        f.write(header.encode("ascii"))
        f.write(bytes(gray_data))

    # YAML 파일 저장
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
    origin_x = map_msg.origin.x
    origin_y = map_msg.origin.y
    origin_theta = map_msg.origin.theta
    resolution = map_msg.resolution if map_msg.resolution > 0.0 else 0.05

    yaml_content = (
        f"image: {map_name}.pgm\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, {origin_theta}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    rospy.loginfo(
        "맵 저장 완료:\n  PGM : %s\n  YAML: %s\n"
        "  크기: %dx%d px, 해상도: %.4f m/px\n"
        "  원점: (%.3f, %.3f, %.4f rad)",
        pgm_path, yaml_path,
        width, height, resolution,
        origin_x, origin_y, origin_theta,
    )
    return True


async def run(robot_ip, robot_port, output_dir, map_name, scene_name_override, map_name_override):
    logger = logging.getLogger("export_map.sdk")
    logger.setLevel(logging.CRITICAL)
    logger.propagate = False
    logger.handlers = []
    logger.addHandler(logging.NullHandler())

    settings = CommuSettings(
        addr=robot_ip,
        port=robot_port,
        identity="map_exporter",
        logger=logger,
        log_level="CRITICAL",
        log_to_console=False,
        log_to_file=False,
    )
    robot = WooshRobot(settings)

    ok = await robot.run()
    if not ok:
        raise RuntimeError("Woosh SDK 연결 실패.")

    info, ok, msg = await robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
    if not ok:
        raise RuntimeError(f"로봇 정보 조회 실패: {msg}")

    target_scene = scene_name_override or info.scene.scene_name
    target_map = map_name_override or info.scene.map_name

    if not target_scene:
        raise RuntimeError("현재 로드된 scene이 없습니다. --scene_name 파라미터를 지정하세요.")

    rospy.loginfo("scene '%s' 맵 데이터를 요청 중...", target_scene)

    req = SceneData()
    req.name = target_scene
    scene_data, ok, msg = await robot.scene_data_req(req, NO_PRINT, NO_PRINT)
    if not ok or not scene_data:
        raise RuntimeError(f"scene_data_req 실패: {msg}")
    if not scene_data.maps:
        raise RuntimeError(f"scene '{target_scene}' 에 맵이 없습니다.")

    # 사용할 맵 선택
    selected_map = None
    if target_map:
        for m in scene_data.maps:
            if m.name == target_map:
                selected_map = m
                break
    if selected_map is None:
        selected_map = scene_data.maps[0]

    rospy.loginfo(
        "선택된 맵: scene=%s, map=%s (id=%d)",
        scene_data.name, selected_map.name, selected_map.id,
    )

    os.makedirs(output_dir, exist_ok=True)
    success = _save_pgm_and_yaml(selected_map, output_dir, map_name)

    await robot.stop()
    return success


def main():
    rospy.init_node("woosh_export_map", anonymous=True)

    robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
    robot_port = rospy.get_param("~robot_port", 5480)
    output_dir = rospy.get_param("~output_dir", "/root/catkin_ws/src/TR-200/woosh_navigation/maps")
    map_name = rospy.get_param("~map_name", "woosh_map")
    scene_name_override = rospy.get_param("~scene_name", "")
    map_name_override = rospy.get_param("~map_name_filter", "")

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        success = loop.run_until_complete(
            run(robot_ip, robot_port, output_dir, map_name, scene_name_override, map_name_override)
        )
        if success:
            rospy.loginfo("맵 내보내기 완료.")
        else:
            rospy.logerr("맵 내보내기 실패.")
    except Exception as exc:
        rospy.logerr("맵 내보내기 오류: %s", exc)
    finally:
        loop.close()


if __name__ == "__main__":
    main()
