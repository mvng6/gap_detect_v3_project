#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RViz debug bridge for Woosh scene loading and localization.

This node connects to the Woosh robot SDK, fetches the currently loaded scene,
converts the SDK map PNG into a ROS OccupancyGrid, and publishes the robot pose,
trajectory, navigation path, scan points, and map markers so the localization
result can be inspected in RViz.

Recommended RViz setup:
  Fixed Frame: map
  Add displays:
    - Map: /woosh_debug/map
    - Pose: /woosh_debug/pose
    - Path: /woosh_debug/trace
    - Path: /woosh_debug/nav_path
    - MarkerArray: /woosh_debug/map_markers
    - MarkerArray: /woosh_debug/dynamic_markers

Example:
  rosrun woosh_bringup woosh_rviz_debug.py _robot_ip:=169.254.128.2 _robot_port:=5480
"""

import asyncio
import io
import math
import os
import sys
from collections import deque

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from PIL import Image
from visualization_msgs.msg import Marker, MarkerArray

# Allow direct execution from source tree.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WOOSH_ROBOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../woosh_robot_py"))
if WOOSH_ROBOT_DIR not in sys.path:
    sys.path.insert(0, WOOSH_ROBOT_DIR)

from woosh_robot import WooshRobot  # noqa: E402
from woosh_interface import CommuSettings, NO_PRINT  # noqa: E402
from woosh.proto.map.map_pack_pb2 import SceneData  # noqa: E402
from woosh.proto.robot.robot_pb2 import (  # noqa: E402
    Model,
    NavPath,
    PoseSpeed,
    RobotInfo,
    ScannerData,
    Scene,
)


class WooshRvizDebugNode:
    def __init__(self):
        self.robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
        self.robot_port = rospy.get_param("~robot_port", 5480)
        self.robot_identity = rospy.get_param("~robot_identity", "rviz_debug")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.topic_prefix = rospy.get_param("~topic_prefix", "/woosh_debug").rstrip("/")
        self.scene_name_override = rospy.get_param("~scene_name", "")
        self.map_name_override = rospy.get_param("~map_name", "")

        self.publish_hz = max(1.0, float(rospy.get_param("~publish_hz", 5.0)))
        self.state_poll_sec = max(0.5, float(rospy.get_param("~state_poll_sec", 1.0)))
        self.scene_refresh_sec = max(1.0, float(rospy.get_param("~scene_refresh_sec", 5.0)))
        self.trace_length = max(10, int(rospy.get_param("~trace_length", 500)))
        self.trace_min_distance = max(0.0, float(rospy.get_param("~trace_min_distance", 0.03)))
        self.publish_scan_points = bool(rospy.get_param("~publish_scan_points", True))
        self.publish_raster_map = bool(rospy.get_param("~publish_raster_map", True))
        self.publish_vector_map = bool(rospy.get_param("~publish_vector_map", True))
        self.footprint_length = float(rospy.get_param("~footprint_length", 0.65))
        self.footprint_width = float(rospy.get_param("~footprint_width", 0.45))

        self.robot = None
        self.last_state_poll = rospy.Time(0)
        self.last_scene_refresh = rospy.Time(0)
        self.scene_dirty = True
        self.cached_scene_key = None

        self.current_scene_name = self.scene_name_override
        self.current_map_name = self.map_name_override
        self.current_map_id = 0

        self.latest_pose = None
        self.latest_nav_path = None
        self.latest_scan = None
        self.latest_model_points = []
        self.initial_localized_pose = None
        self.trace_points = deque(maxlen=self.trace_length)

        self.map_pub = rospy.Publisher(
            f"{self.topic_prefix}/map", OccupancyGrid, queue_size=1, latch=True
        )
        self.pose_pub = rospy.Publisher(
            f"{self.topic_prefix}/pose", PoseStamped, queue_size=10
        )
        self.trace_pub = rospy.Publisher(
            f"{self.topic_prefix}/trace", Path, queue_size=10
        )
        self.nav_path_pub = rospy.Publisher(
            f"{self.topic_prefix}/nav_path", Path, queue_size=10
        )
        self.map_markers_pub = rospy.Publisher(
            f"{self.topic_prefix}/map_markers", MarkerArray, queue_size=1, latch=True
        )
        self.dynamic_markers_pub = rospy.Publisher(
            f"{self.topic_prefix}/dynamic_markers", MarkerArray, queue_size=10
        )

    async def connect(self):
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity,
        )
        self.robot = WooshRobot(settings)

        ok = await self.robot.run()
        if not ok:
            raise RuntimeError("Woosh SDK 연결 시작에 실패했습니다.")

        info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError(f"로봇 정보 조회 실패: {msg}")

        rospy.loginfo(
            "RViz 디버그 노드 연결 성공 (scene=%s, map=%s, map_id=%s)",
            info.scene.scene_name,
            info.scene.map_name,
            info.scene.map_id,
        )

        self._update_scene_state(info.scene)
        self._update_model_state(info.model)
        self._update_pose_state(info.pose_speed)

        await self._subscribe_topics()
        await self._seed_requests()

    async def _subscribe_topics(self):
        subscriptions = [
            ("scene", self.robot.robot_scene_sub, self._on_scene),
            ("pose", self.robot.robot_pose_speed_sub, self._on_pose),
            ("nav_path", self.robot.robot_nav_path_sub, self._on_nav_path),
            ("model", self.robot.robot_model_sub, self._on_model),
        ]
        if self.publish_scan_points:
            subscriptions.append(("scan", self.robot.scanner_data_sub, self._on_scan))

        for name, fn, callback in subscriptions:
            try:
                success = await fn(callback, NO_PRINT)
                if success:
                    rospy.loginfo("구독 성공: %s", name)
                else:
                    rospy.logwarn("구독 실패: %s", name)
            except Exception as exc:
                rospy.logwarn("구독 예외 (%s): %s", name, exc)

    async def _seed_requests(self):
        await self._request_pose_once()
        await self._request_nav_path_once()
        if self.publish_scan_points:
            await self._request_scan_once()
        await self._refresh_scene_data(force=True)

    async def _request_pose_once(self):
        try:
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            if ok and pose_speed:
                self._update_pose_state(pose_speed)
            else:
                rospy.logwarn_throttle(10.0, "PoseSpeed 요청 실패: %s", msg)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "PoseSpeed 요청 예외: %s", exc)

    async def _request_scene_once(self):
        try:
            scene_msg, ok, msg = await self.robot.robot_scene_req(
                Scene(), NO_PRINT, NO_PRINT
            )
            if ok and scene_msg:
                self._update_scene_state(scene_msg)
            else:
                rospy.logwarn_throttle(10.0, "Scene 요청 실패: %s", msg)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "Scene 요청 예외: %s", exc)

    async def _request_nav_path_once(self):
        try:
            nav_path, ok, msg = await self.robot.robot_nav_path_req(
                NavPath(), NO_PRINT, NO_PRINT
            )
            if ok and nav_path:
                self.latest_nav_path = nav_path
            elif msg:
                rospy.loginfo_throttle(15.0, "NavPath 요청 응답: %s", msg)
        except Exception as exc:
            rospy.logwarn_throttle(15.0, "NavPath 요청 예외: %s", exc)

    async def _request_scan_once(self):
        try:
            scan, ok, msg = await self.robot.scanner_data_req(
                ScannerData(), NO_PRINT, NO_PRINT
            )
            if ok and scan:
                self.latest_scan = scan
            elif msg:
                rospy.loginfo_throttle(15.0, "ScannerData 요청 응답: %s", msg)
        except Exception as exc:
            rospy.logwarn_throttle(15.0, "ScannerData 요청 예외: %s", exc)

    def _on_scene(self, scene_msg):
        self._update_scene_state(scene_msg)

    def _on_pose(self, pose_speed):
        self._update_pose_state(pose_speed)

    def _on_nav_path(self, nav_path):
        self.latest_nav_path = nav_path

    def _on_model(self, model):
        self._update_model_state(model)

    def _on_scan(self, scan):
        self.latest_scan = scan

    def _update_scene_state(self, scene_msg):
        new_scene = self.scene_name_override or scene_msg.scene_name
        new_map = self.map_name_override or scene_msg.map_name
        new_map_id = scene_msg.map_id

        changed = (
            new_scene != self.current_scene_name
            or new_map != self.current_map_name
            or new_map_id != self.current_map_id
        )

        self.current_scene_name = new_scene
        self.current_map_name = new_map
        self.current_map_id = new_map_id

        if changed:
            self.initial_localized_pose = None
            self.trace_points.clear()
            self.scene_dirty = True
            rospy.loginfo(
                "현재 scene/map 업데이트: scene=%s, map=%s, map_id=%s",
                self.current_scene_name or "-",
                self.current_map_name or "-",
                self.current_map_id,
            )

    def _update_pose_state(self, pose_speed):
        if not pose_speed:
            return

        previous_map_id = self.current_map_id
        self.latest_pose = pose_speed
        self.current_map_id = pose_speed.map_id

        pose = pose_speed.pose
        if self.initial_localized_pose is None and pose_speed.map_id:
            self.initial_localized_pose = (pose.x, pose.y, pose.theta)

        self._append_trace_point(pose.x, pose.y, pose.theta)

        if previous_map_id != self.current_map_id:
            self.scene_dirty = True

    def _update_model_state(self, model_msg):
        points = []
        if model_msg and model_msg.model:
            for point in model_msg.model:
                points.append((point.x, point.y))

        if not points:
            half_l = self.footprint_length / 2.0
            half_w = self.footprint_width / 2.0
            points = [
                (half_l, half_w),
                (half_l, -half_w),
                (-half_l, -half_w),
                (-half_l, half_w),
            ]

        self.latest_model_points = points

    def _append_trace_point(self, x, y, theta):
        if self.trace_points:
            last_x, last_y, _ = self.trace_points[-1]
            if math.hypot(x - last_x, y - last_y) < self.trace_min_distance:
                return
        self.trace_points.append((x, y, theta))

    async def _refresh_scene_data(self, force=False):
        scene_name = self.scene_name_override or self.current_scene_name
        if not scene_name:
            rospy.loginfo_throttle(10.0, "scene 이름이 아직 없어 맵 데이터를 기다리는 중입니다.")
            return

        now = rospy.Time.now()
        if (
            not force
            and not self.scene_dirty
            and (now - self.last_scene_refresh).to_sec() < self.scene_refresh_sec
        ):
            return

        req = SceneData()
        req.name = scene_name

        try:
            scene_data, ok, msg = await self.robot.scene_data_req(req, NO_PRINT, NO_PRINT)
        except Exception as exc:
            rospy.logwarn("scene_data_req 예외: %s", exc)
            return

        if not ok or not scene_data:
            rospy.logwarn("scene_data_req 실패: %s", msg)
            return
        if not scene_data.maps:
            rospy.logwarn("scene '%s' 에 맵 데이터가 없습니다.", scene_name)
            return

        selected_map = self._select_map(scene_data)
        if selected_map is None:
            rospy.logwarn("scene '%s' 에서 사용할 맵을 선택하지 못했습니다.", scene_name)
            return

        cache_key = (
            scene_data.name,
            selected_map.id,
            selected_map.name,
            selected_map.time_write,
            selected_map.resolution,
            len(selected_map.map_png),
        )
        if cache_key == self.cached_scene_key and not force and not self.scene_dirty:
            self.last_scene_refresh = now
            return

        self.cached_scene_key = cache_key
        self.current_map_name = selected_map.name or self.current_map_name
        self.current_map_id = selected_map.id or self.current_map_id
        self.last_scene_refresh = now
        self.scene_dirty = False

        if self.publish_raster_map:
            occupancy_grid = self._build_occupancy_grid(selected_map)
            if occupancy_grid is not None:
                self.map_pub.publish(occupancy_grid)

        if self.publish_vector_map:
            static_markers = self._build_static_markers(scene_data.name, selected_map)
            self.map_markers_pub.publish(static_markers)

        rospy.loginfo(
            "RViz 디버그 맵 갱신: scene=%s, map=%s, map_id=%s",
            scene_data.name,
            selected_map.name,
            selected_map.id,
        )

    def _select_map(self, scene_data):
        map_name = self.map_name_override or self.current_map_name
        if map_name:
            for map_item in scene_data.maps:
                if map_item.name == map_name:
                    return map_item

        if self.current_map_id:
            for map_item in scene_data.maps:
                if map_item.id == self.current_map_id:
                    return map_item

        return scene_data.maps[0] if scene_data.maps else None

    def _build_occupancy_grid(self, map_msg):
        if not map_msg.map_png:
            rospy.logwarn_throttle(15.0, "map_png 데이터가 없어 OccupancyGrid를 만들 수 없습니다.")
            return None

        base_image = self._open_rgba_image(map_msg.map_png)
        if base_image is None:
            return None

        keepout_image = self._open_rgba_image(map_msg.keepout_png)
        traffic_keepout_image = self._open_rgba_image(map_msg.traffic_keepout_png)

        width, height = base_image.size
        base_pixels = base_image.load()
        keepout_pixels = keepout_image.load() if keepout_image else None
        traffic_pixels = traffic_keepout_image.load() if traffic_keepout_image else None

        data = []
        for y in range(height - 1, -1, -1):
            for x in range(width):
                r, g, b, a = base_pixels[x, y]
                if a < 10:
                    occ = -1
                else:
                    intensity = (float(r) + float(g) + float(b)) / 3.0
                    occ = int(round((255.0 - intensity) / 255.0 * 100.0))
                    occ = max(0, min(100, occ))

                if keepout_pixels is not None:
                    kr, kg, kb, ka = keepout_pixels[x, y]
                    if ka > 10 and (kr + kg + kb) < 700:
                        occ = 100

                if traffic_pixels is not None:
                    tr, tg, tb, ta = traffic_pixels[x, y]
                    if ta > 10 and (tr + tg + tb) < 700:
                        occ = 100

                data.append(occ)

        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.map_frame
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = map_msg.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = map_msg.origin.x
        grid.info.origin.position.y = map_msg.origin.y
        qz, qw = self._yaw_to_quaternion(map_msg.origin.theta)
        grid.info.origin.orientation.z = qz
        grid.info.origin.orientation.w = qw
        grid.data = data
        return grid

    def _open_rgba_image(self, png_bytes):
        if not png_bytes:
            return None

        try:
            with Image.open(io.BytesIO(png_bytes)) as image:
                return image.convert("RGBA")
        except Exception as exc:
            rospy.logwarn("PNG 디코딩 실패: %s", exc)
            return None

    def _build_static_markers(self, scene_name, map_msg):
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker())

        marker_id = 1

        for bidirect_path in map_msg.path_info.bidirect_paths:
            marker_array.markers.append(
                self._line_strip_marker(
                    marker_id,
                    "bidirect_path",
                    bidirect_path.path,
                    0.06,
                    (0.15, 0.75, 1.0, 0.95),
                )
            )
            marker_id += 1

        for mono_path in map_msg.path_info.mono_paths:
            marker_array.markers.append(
                self._line_strip_marker(
                    marker_id,
                    "mono_path",
                    mono_path.path,
                    0.05,
                    (1.0, 0.65, 0.1, 0.95),
                )
            )
            marker_id += 1

        for storage in map_msg.mark_info.storages:
            pose = self._extract_storage_pose(storage)
            if pose is None:
                continue

            color = self._storage_color(storage)
            label = self._storage_label(storage)

            marker_array.markers.append(
                self._cube_marker(
                    marker_id,
                    "storage",
                    pose[0],
                    pose[1],
                    pose[2],
                    (0.35, 0.25, 0.20),
                    color,
                )
            )
            marker_id += 1

            if label:
                marker_array.markers.append(
                    self._text_marker(
                        marker_id,
                        "storage_label",
                        pose[0],
                        pose[1],
                        0.35,
                        label,
                        0.20,
                        (1.0, 1.0, 1.0, 0.95),
                    )
                )
                marker_id += 1

        for artag in map_msg.mark_info.artags:
            marker_array.markers.append(
                self._sphere_marker(
                    marker_id,
                    "artag",
                    artag.pose.x,
                    artag.pose.y,
                    0.06,
                    0.12,
                    (0.95, 0.2, 0.2, 0.95),
                )
            )
            marker_id += 1

        for reflector in map_msg.mark_info.reflectors:
            marker_array.markers.append(
                self._sphere_marker(
                    marker_id,
                    "reflector",
                    reflector.pose.x,
                    reflector.pose.y,
                    0.06,
                    0.10,
                    (0.8, 0.8, 0.8, 0.95),
                )
            )
            marker_id += 1

        for beacon in map_msg.mark_info.beacons:
            marker_array.markers.append(
                self._sphere_marker(
                    marker_id,
                    "beacon",
                    beacon.point.x,
                    beacon.point.y,
                    0.06,
                    0.10,
                    (0.15, 1.0, 0.35, 0.95),
                )
            )
            marker_id += 1

        marker_array.markers.append(
            self._text_marker(
                marker_id,
                "scene_info",
                map_msg.origin.x,
                map_msg.origin.y,
                0.55,
                f"scene: {scene_name}\nmap: {map_msg.name}\nmap_id: {map_msg.id}",
                0.25,
                (0.95, 0.95, 0.95, 0.95),
            )
        )

        return marker_array

    def _build_dynamic_markers(self):
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker())

        marker_id = 1

        if self.latest_pose is not None:
            pose = self.latest_pose.pose

            marker_array.markers.append(
                self._arrow_marker(
                    marker_id,
                    "robot_pose",
                    pose.x,
                    pose.y,
                    pose.theta,
                    (0.55, 0.16, 0.16),
                    (0.1, 1.0, 0.2, 0.95),
                )
            )
            marker_id += 1

            footprint_points = self._transform_footprint_points(
                pose.x, pose.y, pose.theta, self.latest_model_points
            )
            marker_array.markers.append(
                self._line_strip_marker(
                    marker_id,
                    "robot_footprint",
                    footprint_points,
                    0.04,
                    (0.1, 1.0, 0.2, 0.95),
                    closed=True,
                )
            )
            marker_id += 1

            if self.initial_localized_pose is not None:
                init_x, init_y, init_theta = self.initial_localized_pose
                marker_array.markers.append(
                    self._arrow_marker(
                        marker_id,
                        "initial_localized_pose",
                        init_x,
                        init_y,
                        init_theta,
                        (0.40, 0.12, 0.12),
                        (0.1, 0.7, 1.0, 0.85),
                    )
                )
                marker_id += 1

            marker_array.markers.append(
                self._text_marker(
                    marker_id,
                    "robot_status",
                    pose.x,
                    pose.y,
                    0.55,
                    self._build_status_text(),
                    0.22,
                    (1.0, 1.0, 1.0, 0.95),
                )
            )
            marker_id += 1

        if self.publish_scan_points and self.latest_scan is not None:
            scan_points = self._scan_points_in_map(self.latest_scan)
            if scan_points:
                marker_array.markers.append(
                    self._points_marker(
                        marker_id,
                        "scan_points",
                        scan_points,
                        0.04,
                        (1.0, 0.15, 0.15, 0.80),
                    )
                )

        return marker_array

    def _build_status_text(self):
        if self.latest_pose is None:
            return "pose: waiting"

        pose = self.latest_pose.pose
        localized = "yes" if self.latest_pose.map_id else "no"
        return (
            f"scene: {self.current_scene_name or '-'}\n"
            f"map: {self.current_map_name or '-'} ({self.current_map_id})\n"
            f"localized: {localized}\n"
            f"x={pose.x:.2f}, y={pose.y:.2f}, th={pose.theta:.2f}"
        )

    def _publish_pose(self):
        if self.latest_pose is None:
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = self.latest_pose.pose.x
        msg.pose.position.y = self.latest_pose.pose.y
        qz, qw = self._yaw_to_quaternion(self.latest_pose.pose.theta)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _publish_trace(self):
        if not self.trace_points:
            return

        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame

        for x, y, theta in self.trace_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qz, qw = self._yaw_to_quaternion(theta)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)

        self.trace_pub.publish(msg)

    def _publish_nav_path(self):
        if self.latest_nav_path is None:
            return

        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame

        for pose2d in self.latest_nav_path.path.poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pose2d.x
            pose.pose.position.y = pose2d.y
            qz, qw = self._yaw_to_quaternion(pose2d.theta)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)

        self.nav_path_pub.publish(msg)

    def _scan_points_in_map(self, scan_msg):
        points = []
        base_theta = scan_msg.pose.theta
        angle = scan_msg.angle_min

        for distance in scan_msg.ranges:
            if scan_msg.range_min <= distance <= scan_msg.range_max:
                points.append(
                    (
                        scan_msg.pose.x + distance * math.cos(base_theta + angle),
                        scan_msg.pose.y + distance * math.sin(base_theta + angle),
                    )
                )
            angle += scan_msg.angle_increment

        return points

    def _transform_footprint_points(self, x, y, theta, points):
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        transformed = []
        for px, py in points:
            transformed.append(
                (
                    x + cos_theta * px - sin_theta * py,
                    y + sin_theta * px + cos_theta * py,
                )
            )
        return transformed

    def _extract_storage_pose(self, storage):
        pose_msg = None
        if storage.HasField("pose"):
            pose_msg = storage.pose.real
        if pose_msg is None:
            return None
        return pose_msg.x, pose_msg.y, pose_msg.theta

    def _storage_color(self, storage):
        if storage.HasField("charger"):
            return 0.10, 0.85, 0.25, 0.95
        if storage.HasField("parkspot"):
            return 0.20, 0.55, 1.0, 0.95
        if storage.HasField("rack"):
            return 1.0, 0.75, 0.20, 0.95
        if storage.HasField("roller_station"):
            return 0.95, 0.35, 0.20, 0.95
        return 0.75, 0.75, 0.75, 0.95

    def _storage_label(self, storage):
        if storage.HasField("identity"):
            if storage.identity.no:
                return storage.identity.no
            if storage.identity.desc:
                return storage.identity.desc
        if storage.HasField("charger"):
            return "charger"
        if storage.HasField("parkspot"):
            return "parkspot"
        if storage.HasField("rack"):
            return "rack"
        return ""

    def _delete_all_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        return marker

    def _line_strip_marker(self, marker_id, namespace, points, width, color, closed=False):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        for point in points:
            x, y = self._xy_from_point_like(point)
            marker.points.append(Point(x=x, y=y, z=0.03))

        if closed and marker.points:
            marker.points.append(marker.points[0])

        return marker

    def _points_marker(self, marker_id, namespace, points, scale, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        for x, y in points:
            marker.points.append(Point(x=x, y=y, z=0.04))

        return marker

    def _sphere_marker(self, marker_id, namespace, x, y, z, size, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _cube_marker(self, marker_id, namespace, x, y, theta, scale_xyz, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = scale_xyz[2] / 2.0
        qz, qw = self._yaw_to_quaternion(theta)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x, marker.scale.y, marker.scale.z = scale_xyz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _arrow_marker(self, marker_id, namespace, x, y, theta, scale_xyz, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        qz, qw = self._yaw_to_quaternion(theta)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x, marker.scale.y, marker.scale.z = scale_xyz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _text_marker(self, marker_id, namespace, x, y, z, text, scale_z, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale_z
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    def _xy_from_point_like(self, point):
        if isinstance(point, tuple):
            return point[0], point[1]
        return point.x, point.y

    def _yaw_to_quaternion(self, yaw):
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    async def spin(self):
        rate = 1.0 / self.publish_hz
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_state_poll).to_sec() >= self.state_poll_sec:
                self.last_state_poll = now
                await self._request_scene_once()
                await self._request_pose_once()
                await self._request_nav_path_once()
                if self.publish_scan_points:
                    await self._request_scan_once()
            await self._refresh_scene_data()
            self._publish_pose()
            self._publish_trace()
            self._publish_nav_path()
            self.dynamic_markers_pub.publish(self._build_dynamic_markers())
            await asyncio.sleep(rate)

    async def shutdown(self):
        if self.robot is not None:
            try:
                await self.robot.stop()
            except Exception as exc:
                rospy.logwarn("SDK 종료 중 예외: %s", exc)


def main():
    rospy.init_node("woosh_rviz_debug", anonymous=False)
    node = WooshRvizDebugNode()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(node.connect())
        loop.run_until_complete(node.spin())
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료되었습니다.")
    except Exception as exc:
        rospy.logerr("RViz 디버그 노드 예외: %s", exc)
    finally:
        loop.run_until_complete(node.shutdown())
        loop.close()


if __name__ == "__main__":
    main()
