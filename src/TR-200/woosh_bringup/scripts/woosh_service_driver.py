#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import asyncio
import csv
import math
import numpy as np
import sys
import os
import shutil
import signal
import subprocess
import atexit
import time
from queue import Queue, Empty
from threading import Thread, Event, Lock

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped, Twist as RosTwist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

# === Python 경로 설정 ===
script_dir = os.path.dirname(os.path.abspath(__file__))

woosh_robot_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_robot_py"))
if woosh_robot_dir not in sys.path:
    sys.path.insert(0, woosh_robot_dir)

try:
    from woosh_utils import (
        clear_registered_sdk_owner,
        current_process_is_registered_owner,
        find_foreign_sdk_owners,
        inspect_tcp_connections,
        log_sdk_owner,
        parse_connection_owners,
        print_battery_status,
        register_sdk_owner,
    )
except ImportError:
    woosh_utils_src_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_utils/src"))
    if woosh_utils_src_dir not in sys.path:
        sys.path.insert(0, woosh_utils_src_dir)
    from woosh_utils import (
        clear_registered_sdk_owner,
        current_process_is_registered_owner,
        find_foreign_sdk_owners,
        inspect_tcp_connections,
        log_sdk_owner,
        parse_connection_owners,
        print_battery_status,
        register_sdk_owner,
    )

from woosh_msgs.srv import MoveMobile, MoveMobileResponse
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pack_pb2 import Twist, SwitchMap, SetRobotPose, InitRobot, SwitchControlMode
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed, OperationState, ScannerData
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.robot_pb2 import ControlMode


DEFAULT_MAP_FILE = "/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml"
DEFAULT_CARTO_STATE_FILE = "/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream"


def _yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


# ---------------------------------------------------------------------------
# 파일 탐색 유틸리티
# ---------------------------------------------------------------------------

def _find_package_dir():
    source_candidate = os.path.abspath(os.path.join(script_dir, ".."))
    if os.path.isfile(os.path.join(source_candidate, "package.xml")):
        return source_candidate
    try:
        import rospkg
        return rospkg.RosPack().get_path("woosh_bringup")
    except Exception:
        return source_candidate


def _resolve_file(*candidates, rospkg_fallback=None):
    """후보 경로 목록에서 존재하는 첫 번째 파일을 반환한다.

    Args:
        *candidates: 절대 경로 문자열들
        rospkg_fallback: (package_name, relative_path) 튜플. 후보가 모두 없을 때
                         rospkg를 이용해 탐색한다.
    """
    for path in candidates:
        if os.path.isfile(path):
            return path

    if rospkg_fallback:
        try:
            import rospkg
            pkg_name, rel_path = rospkg_fallback
            return os.path.join(rospkg.RosPack().get_path(pkg_name), rel_path)
        except Exception:
            pass

    return None


def _find_rviz_config():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../rviz/woosh_rviz_debug.rviz")),
        os.path.join(_find_package_dir(), "rviz", "woosh_rviz_debug.rviz"),
    )


def _find_amcl_rviz_config():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/AMCL/rviz/amcl_debug.rviz")),
        rospkg_fallback=("woosh_slam_amcl", os.path.join("rviz", "amcl_debug.rviz")),
    )


def _rviz_config_contains_topic(config_path, topic_name):
    """RViz 설정 파일에 지정 토픽 문자열이 포함되어 있는지 검사한다."""
    if not config_path:
        return False
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            return topic_name in f.read()
    except Exception as exc:
        rospy.logwarn("RViz 설정 파일 검사 실패 (%s): %s", config_path, exc)
        return False


def _find_local_maps_dir():
    """프로젝트 내 woosh_slam/maps 디렉터리를 반환한다."""
    candidates = [
        os.path.abspath(os.path.join(script_dir, "../../woosh_slam/maps")),
    ]
    try:
        import rospkg
        pkg_dir = rospkg.RosPack().get_path("woosh_slam_gmapping")
        candidates.append(os.path.abspath(os.path.join(pkg_dir, "..", "..", "maps")))
    except Exception:
        pass
    for d in candidates:
        if os.path.isdir(d):
            return d
    return None


def _find_yaml_for_map(map_name):
    """맵 이름에 대응하는 .yaml 파일 경로를 로컬 맵 목록에서 탐색한다.

    Returns:
        str or None: .yaml 파일의 절대 경로, 없으면 None
    """
    for m in _get_local_map_names():
        if m["name"] == map_name:
            return m["path"]
    return None


def _find_pbstream_for_map(map_name):
    """맵 이름에 대응하는 .pbstream 파일을 탐색한다.

    woosh_slam/maps 디렉터리에서 map_name.pbstream 파일을 찾는다.

    Returns:
        str or None: .pbstream 파일의 절대 경로, 없으면 None
    """
    maps_dir = _find_local_maps_dir()
    if maps_dir is None:
        return None

    pbstream_path = os.path.join(maps_dir, f"{map_name}.pbstream")
    if os.path.isfile(pbstream_path):
        return pbstream_path

    # carto_ 접두사가 붙은 파일도 탐색 (예: carto_map_name.pbstream)
    carto_path = os.path.join(maps_dir, f"carto_{map_name}.pbstream")
    if os.path.isfile(carto_path):
        return carto_path

    return None


def _find_yaml_for_state_file(state_file):
    """`.pbstream` 파일과 동일한 basename의 `.yaml` 파일을 탐색한다."""
    if not state_file:
        return None

    base, ext = os.path.splitext(state_file)
    if ext.lower() != ".pbstream":
        return None

    yaml_path = base + ".yaml"
    if os.path.isfile(yaml_path):
        return yaml_path

    return None


def _get_local_map_names():
    """woosh_slam/maps 디렉터리에서 유효한 맵 이름 목록을 반환한다.

    Returns:
        list of dict: [{"name": str, "path": str}, ...]
            name — .yaml 확장자를 제거한 파일명
            path — .yaml 파일의 절대 경로
    """
    maps_dir = _find_local_maps_dir()
    if maps_dir is None:
        return []

    result = []
    try:
        for fname in sorted(os.listdir(maps_dir)):
            if not fname.endswith(".yaml"):
                continue
            yaml_path = os.path.join(maps_dir, fname)
            pgm_path = yaml_path[:-5] + ".pgm"
            if not os.path.isfile(pgm_path):
                continue
            result.append({"name": os.path.splitext(fname)[0], "path": yaml_path})
    except Exception as exc:
        rospy.logwarn("로컬 맵 디렉터리 스캔 실패: %s", exc)

    return result


def _find_amcl_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/AMCL/launch/amcl.launch")),
        rospkg_fallback=("woosh_slam_amcl", os.path.join("launch", "amcl.launch")),
    )


def _find_gmapping_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_slam/GMapping/woosh_slam_gmapping/launch/gmapping.launch")),
        rospkg_fallback=("woosh_slam_gmapping", os.path.join("launch", "gmapping.launch")),
    )


def _find_cartographer_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_slam/Cartographer/woosh_slam_cartographer/launch/cartographer.launch")),
        rospkg_fallback=("woosh_slam_cartographer", os.path.join("launch", "cartographer.launch")),
    )


def _find_cartographer_localization_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_slam/Cartographer/woosh_slam_cartographer/launch/cartographer_localization.launch")),
        rospkg_fallback=("woosh_slam_cartographer", os.path.join("launch", "cartographer_localization.launch")),
    )


def _find_costmap_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/Costmap/woosh_costmap/launch/global_costmap.launch")),
        rospkg_fallback=("woosh_costmap", os.path.join("launch", "global_costmap.launch")),
    )


def _find_costmap_rviz_config():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/Costmap/woosh_costmap/rviz/costmap_debug.rviz")),
        rospkg_fallback=("woosh_costmap", os.path.join("rviz", "costmap_debug.rviz")),
    )


def _find_move_base_only_launch():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/MoveBase/woosh_navigation_mb/launch/move_base_only.launch")),
        rospkg_fallback=("woosh_navigation_mb", os.path.join("launch", "move_base_only.launch")),
    )


def _find_rviz_binary():
    return shutil.which("rviz") or (
        "/opt/ros/noetic/bin/rviz"
        if os.path.isfile("/opt/ros/noetic/bin/rviz")
        else None
    )


def _validate_map_file(map_file):
    if not os.path.isfile(map_file):
        rospy.logerr("맵 파일을 찾을 수 없습니다: %s", map_file)
        rospy.logerr("  원인: Docker 볼륨 마운트가 적용되지 않았거나 컨테이너 밖에서 실행 중일 수 있습니다.")
        rospy.logerr("  해결:")
        rospy.logerr("    1. 컨테이너 재시작: docker-compose -f docker-compose.noetic_integration.yml up -d")
        rospy.logerr("    2. 컨테이너 진입:   docker exec -it noetic_robot_system_ws bash")
        rospy.logerr("    3. 맵 생성:         rosrun woosh_gmap_amcl export_map.py _robot_ip:=169.254.128.2")
        rospy.logerr("    4. 다른 맵 지정:    rosrun woosh_bringup woosh_service_driver.py amcl map_file:=/path/to/map.yaml")
        return False

    try:
        map_dir = os.path.dirname(map_file)
        with open(map_file) as f:
            for line in f:
                if line.startswith("image:"):
                    img_path = line.split(":", 1)[1].strip()
                    if not os.path.isabs(img_path):
                        img_path = os.path.join(map_dir, img_path)
                    if not os.path.isfile(img_path):
                        rospy.logerr("맵 이미지 파일을 찾을 수 없습니다: %s", img_path)
                        return False
                    break
    except Exception as exc:
        rospy.logwarn("맵 파일 내용 검증 중 오류 (계속 진행): %s", exc)

    return True


def _wait_for_topic_message(topic_name, msg_type, timeout, label=None):
    """지정 토픽에서 메시지 1개를 받을 때까지 대기한다."""
    if timeout <= 0.0:
        return False

    wait_label = label or topic_name
    rospy.loginfo("%s 준비 대기 중... (최대 %.1fs)", wait_label, timeout)

    try:
        rospy.wait_for_message(topic_name, msg_type, timeout=timeout)
        rospy.loginfo("%s 확인 완료", wait_label)
        return True
    except rospy.ROSException:
        rospy.logwarn("%s 대기 타임아웃 (%s)", wait_label, topic_name)
        return False


def _wait_for_transform(target_frame, source_frame, timeout, label=None, poll_period=0.2):
    """TF 변환이 조회 가능해질 때까지 대기한다."""
    if timeout <= 0.0:
        return False

    wait_label = label or f"TF {target_frame} -> {source_frame}"
    rospy.loginfo("%s 준비 대기 중... (최대 %.1fs)", wait_label, timeout)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    deadline = time.monotonic() + timeout
    ok = False

    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            wait_slice = max(min(remaining, poll_period), 0.05)
            try:
                if tf_buffer.can_transform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(wait_slice),
                ):
                    ok = True
                    break
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                pass
    finally:
        del listener

    if ok:
        rospy.loginfo("%s 확인 완료", wait_label)
    else:
        rospy.logwarn("%s 대기 타임아웃", wait_label)

    return ok


def _get_published_topic_map():
    try:
        return dict(rospy.get_published_topics())
    except Exception as exc:
        rospy.logwarn("발행 토픽 목록 조회 실패: %s", exc)
        return {}


def _log_nav_readiness_diagnostics(localization_mode=None):
    """nav_on 준비 실패 시 원인을 빠르게 좁히기 위한 진단 로그."""
    published_topics = _get_published_topic_map()
    topics_to_check = [
        "/scan",
        "/odom",
        "/map",
        "/global_costmap/costmap",
    ]

    if localization_mode == "amcl":
        topics_to_check.append("/amcl_pose")

    rospy.logwarn("=== nav_on 진단 시작 ===")
    for topic_name in topics_to_check:
        topic_type = published_topics.get(topic_name)
        if topic_type:
            rospy.logwarn("토픽 확인: %s (%s)", topic_name, topic_type)
        else:
            rospy.logwarn("토픽 미발견: %s", topic_name)

    tf_ok = _wait_for_transform(
        "map",
        "base_link",
        timeout=0.5,
        label="진단용 TF map -> base_link",
        poll_period=0.1,
    )
    if not tf_ok:
        rospy.logwarn("TF map -> base_link 가 아직 준비되지 않았습니다.")

    if localization_mode == "amcl" and "/amcl_pose" not in published_topics:
        rospy.logwarn("AMCL pose가 보이지 않습니다. /scan, /odom, 초기 pose 상태를 함께 확인하세요.")

    rospy.logwarn("=== nav_on 진단 종료 ===")


# ---------------------------------------------------------------------------
# 서브프로세스 관리
# ---------------------------------------------------------------------------

class SubprocessManager:
    """자식 프로세스의 생성/종료를 관리한다."""

    def __init__(self):
        self._procs = {}  # name -> Popen
        self._lock = Lock()
        self._stopping = set()

    def _monitor(self, name, proc):
        return_code = proc.wait()

        with self._lock:
            stopping = name in self._stopping
            current = self._procs.get(name)
            if current is proc:
                self._procs.pop(name, None)
            if stopping:
                self._stopping.discard(name)

        if rospy.is_shutdown() or stopping:
            rospy.loginfo("%s 종료됨 (exit=%s)", name, return_code)
        elif return_code == 0:
            rospy.logwarn("%s 프로세스가 종료되었습니다. (exit=%s)", name, return_code)
        else:
            rospy.logerr("%s 프로세스가 비정상 종료되었습니다. (exit=%s)", name, return_code)

    def start(self, name, cmd, startup_grace_sec=1.5):
        try:
            proc = subprocess.Popen(cmd, start_new_session=True)
            with self._lock:
                self._procs[name] = proc
                self._stopping.discard(name)

            Thread(target=self._monitor, args=(name, proc), daemon=True).start()

            if startup_grace_sec > 0.0:
                time.sleep(startup_grace_sec)
                if proc.poll() is not None:
                    rospy.logerr("%s 시작 직후 종료됨: %s", name, " ".join(cmd))
                    return False

            rospy.loginfo("%s 시작", name)
            return True
        except Exception as exc:
            rospy.logwarn("%s 시작 실패: %s", name, exc)
            return False

    def stop(self, name):
        with self._lock:
            proc = self._procs.pop(name, None)
            if proc is not None:
                self._stopping.add(name)
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            rospy.loginfo("%s 종료 요청 완료", name)
        except ProcessLookupError:
            pass
        except Exception as exc:
            rospy.logwarn("%s 종료 중 예외: %s", name, exc)

    def stop_all(self):
        for name in list(self._procs):
            self.stop(name)


# ---------------------------------------------------------------------------
# 외부 스택 실행기
# ---------------------------------------------------------------------------

class StackLauncher:
    """센서 브릿지, RViz, SLAM 등 외부 스택 실행을 담당한다."""

    def __init__(self, robot_ip, robot_port):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self._pm = SubprocessManager()

    def shutdown(self):
        self._pm.stop_all()

    # -- base_link→laser 정적 TF (기본 모드 전용) --

    def start_base_laser_tf(
        self,
        base_frame="base_link",
        laser_frame="laser",
        x=0.0, y=0.0, z=0.25, yaw=0.0,
    ):
        """SLAM/localization 스택이 없는 기본 모드에서 base_link→laser TF를 발행한다.

        SLAM/localization 스택이 활성화된 경우에는 해당 스택의 launch 파일이
        static_transform_publisher를 담당하므로 이 메서드를 호출하지 않는다.
        """
        cmd = [
            "rosrun", "tf2_ros", "static_transform_publisher",
            str(x), str(y), str(z),
            str(yaw), "0.0", "0.0",
            base_frame, laser_frame,
        ]
        started = self._pm.start("base_link→laser TF", cmd)
        if started:
            rospy.loginfo("base_link→laser 정적 TF 발행 시작 (기본 모드)")
        return started

    # -- 센서 브릿지 --

    def start_sensor_bridge(self):
        rospy.loginfo(
            "외부 센서 브릿지 서브프로세스는 비활성화되었습니다. "
            "SmoothTwistController가 /scan, /odom, /odom_raw, TF를 직접 발행합니다."
        )
        return True

    def wait_for_nav_prerequisites(self, localization_mode, slam_mode="none", timeout=20.0):
        """nav_on / move_base 시작 전 필수 토픽/TF가 준비될 때까지 대기한다.

        Args:
            localization_mode: 정식 모드 문자열 (none|amcl|carto_fix|carto_nonfix)
                               또는 레거시 문자열 (carto_loc_fix|carto_loc_nonfix)
            slam_mode: 정식 모드 문자열 (none|gmapping|cartographer)
            timeout: 전체 대기 타임아웃 (초)
        """
        deadline = time.monotonic() + timeout
        all_ok = True

        def remaining_time():
            return max(deadline - time.monotonic(), 0.0)

        required_topics = [
            ("/scan", LaserScan, "LiDAR /scan"),
            ("/odom", Odometry, "Odometry /odom"),
        ]

        # SLAM 모드: /map은 SLAM 노드가 발행 (첫 루프 클로저/서브맵 후)
        # Localization 모드: /map은 map_server가 발행 (즉시)
        if slam_mode in ("gmapping", "cartographer"):
            rospy.loginfo("SLAM 모드(%s): /map 및 TF map→odom 대기 중 (최대 %.0f초)...", slam_mode, timeout)
            required_topics.append(("/map", OccupancyGrid, "SLAM Map /map"))
        elif localization_mode in ("amcl", "carto_fix", "carto_loc_fix"):
            required_topics.append(("/map", OccupancyGrid, "Map /map"))
        elif localization_mode in ("carto_nonfix", "carto_loc_nonfix"):
            # nonfix: cartographer_occupancy_grid_node는 /carto_map으로 발행
            # /map(map_server)은 costmap 기동 시 시작되므로 /carto_map을 확인
            required_topics.append(("/carto_map", OccupancyGrid, "Cartographer Map /carto_map"))

        for topic_name, msg_type, label in required_topics:
            remaining = remaining_time()
            if remaining <= 0.0:
                all_ok = False
                break
            ok = _wait_for_topic_message(
                topic_name,
                msg_type,
                timeout=min(remaining, 8.0),
                label=label,
            )
            all_ok = all_ok and ok

        # TF map→odom 대기 (SLAM / Localization 공통)
        if slam_mode in ("gmapping", "cartographer"):
            remaining = remaining_time()
            if remaining > 0.0:
                tf_ok = _wait_for_transform(
                    "map", "odom",
                    timeout=min(remaining, 10.0),
                    label="TF map -> odom (SLAM)",
                )
                all_ok = all_ok and tf_ok
            else:
                all_ok = False

        elif localization_mode == "amcl":
            remaining = remaining_time()
            pose_ok = False
            if remaining > 0.0:
                pose_ok = _wait_for_topic_message(
                    "/amcl_pose",
                    PoseWithCovarianceStamped,
                    timeout=min(remaining, 8.0),
                    label="AMCL pose /amcl_pose",
                )

            remaining = remaining_time()
            tf_ok = False
            if remaining > 0.0:
                tf_ok = _wait_for_transform(
                    "map",
                    "base_link",
                    timeout=min(remaining, 6.0),
                    label="TF map -> base_link",
                )

            all_ok = all_ok and (pose_ok or tf_ok)

        elif localization_mode in ("carto_fix", "carto_nonfix", "carto_loc_fix", "carto_loc_nonfix"):
            remaining = remaining_time()
            if remaining > 0.0:
                tf_ok = _wait_for_transform(
                    "map",
                    "base_link",
                    timeout=min(remaining, 6.0),
                    label="TF map -> base_link",
                )
                all_ok = all_ok and tf_ok
            else:
                all_ok = False

        return all_ok

    def wait_for_costmap_ready(self, localization_mode=None, timeout=20.0):
        """Global Costmap 토픽이 실제로 나오기 시작하는지 확인한다."""
        ready = _wait_for_topic_message(
            "/global_costmap/costmap",
            OccupancyGrid,
            timeout=timeout,
            label="Global Costmap /global_costmap/costmap",
        )
        if not ready:
            rospy.logerr("Global Costmap 토픽이 준비되지 않았습니다.")
            _log_nav_readiness_diagnostics(localization_mode=localization_mode)
        return ready

    # -- RViz --

    def start_rviz(self, use_amcl_rviz=False, require_nav_costmap=False):
        rviz_bin = _find_rviz_binary()

        if use_amcl_rviz:
            rviz_config = _find_amcl_rviz_config()
            config_label = "amcl_debug.rviz"
        else:
            rviz_config = _find_rviz_config()
            config_label = "woosh_rviz_debug.rviz"

        # AMCL + nav_on 환경에서 기존 RViz 설정에 costmap 디스플레이가 없으면
        # costmap 전용 설정으로 폴백하여 시각화 누락을 방지한다.
        if use_amcl_rviz and require_nav_costmap:
            has_costmap_topic = _rviz_config_contains_topic(rviz_config, "/global_costmap/costmap")
            if not has_costmap_topic:
                fallback = _find_costmap_rviz_config()
                if fallback:
                    rospy.logwarn(
                        "amcl_debug.rviz에 /global_costmap/costmap 표시가 없어 costmap_debug.rviz로 대체합니다."
                    )
                    rviz_config = fallback
                    config_label = "costmap_debug.rviz (fallback)"
                else:
                    rospy.logwarn(
                        "amcl_debug.rviz에 costmap 표시가 없고 대체 RViz 설정도 찾지 못했습니다."
                    )

        if not rviz_config:
            rospy.logwarn("RViz 설정 파일(%s)을 찾지 못해 RViz 지원을 시작하지 않습니다.", config_label)
            return False
        if not rviz_bin:
            rospy.logwarn("`rviz` 실행 파일을 찾지 못했습니다. `rviz_on` 요청은 건너뜁니다.")
            return False

        rviz_ok = self._pm.start("RViz", [rviz_bin, "-d", rviz_config])
        if rviz_ok:
            if not use_amcl_rviz:
                rospy.logwarn(
                    "woosh_rviz_debug.py direct SDK bridge는 single-owner 정책으로 비활성화되었습니다. "
                    "woosh_rviz_debug.rviz의 /woosh_debug/* 디스플레이는 비어 있을 수 있습니다."
                )
            rospy.loginfo(
                "RViz 시작 (설정: %s, SDK direct debug bridge 비활성화 — single-owner 정책)",
                config_label,
            )
        return rviz_ok

    # -- AMCL --

    def start_amcl(self, map_file):
        if not _validate_map_file(map_file):
            rospy.logerr("AMCL을 시작하지 않습니다. 위 오류를 해결한 후 재시작하세요.")
            return False

        amcl_launch = _find_amcl_launch()
        if amcl_launch is None:
            rospy.logwarn("amcl.launch 파일을 찾을 수 없습니다. AMCL을 시작하지 않습니다.")
            return False

        cmd = [
            "roslaunch", amcl_launch,
            f"robot_ip:={self.robot_ip}",
            f"robot_port:={self.robot_port}",
            f"map_file:={map_file}",
            "launch_rviz:=false",
            "launch_sensor_bridge:=false",
        ]

        amcl_rviz_config = _find_amcl_rviz_config()
        if amcl_rviz_config:
            cmd.append(f"rviz_config:={amcl_rviz_config}")

        started = self._pm.start("AMCL 스택", cmd)
        if started:
            rospy.loginfo("AMCL 스택 시작 (map_file=%s)", map_file)
        return started

    # -- Global Costmap --

    def start_costmap(
        self,
        map_file,
        launch_map_server=True,
        launch_map_odom_tf=False,
        launch_base_laser_tf=True,
    ):
        if launch_map_server and not _validate_map_file(map_file):
            rospy.logerr("Global Costmap을 시작하지 않습니다. 위 오류를 해결한 후 재시작하세요.")
            return False

        costmap_launch = _find_costmap_launch()
        if costmap_launch is None:
            rospy.logwarn("global_costmap.launch 파일을 찾을 수 없습니다. Global Costmap을 시작하지 않습니다.")
            return False

        cmd = [
            "roslaunch", costmap_launch,
            f"robot_ip:={self.robot_ip}",
            f"robot_port:={self.robot_port}",
            f"map_file:={map_file}",
            "launch_rviz:=false",
            "launch_sensor_bridge:=false",
            f"launch_map_server:={'true' if launch_map_server else 'false'}",
            f"launch_map_odom_tf:={'true' if launch_map_odom_tf else 'false'}",
            f"launch_base_laser_tf:={'true' if launch_base_laser_tf else 'false'}",
        ]

        costmap_rviz_config = _find_costmap_rviz_config()
        if costmap_rviz_config:
            cmd.append(f"rviz_config:={costmap_rviz_config}")

        started = self._pm.start("Global Costmap 스택", cmd)
        if started:
            rospy.loginfo("Global Costmap 시작 (map_file=%s)", map_file)
        return started

    # -- SLAM 공통 --

    def _start_slam_stack(self, name, find_launch_fn, extra_args=None):
        launch_file = find_launch_fn()
        if launch_file is None:
            rospy.logwarn("%s launch 파일을 찾을 수 없습니다.", name)
            return False

        cmd = [
            "roslaunch", launch_file,
            f"robot_ip:={self.robot_ip}",
            f"robot_port:={self.robot_port}",
            "launch_rviz:=true",
            "launch_sensor_bridge:=false",
        ]
        if extra_args:
            cmd.extend(extra_args)

        return self._pm.start(f"{name} 스택", cmd)

    def start_gmapping(self):
        return self._start_slam_stack("GMapping", _find_gmapping_launch)

    def start_cartographer(self):
        return self._start_slam_stack("Cartographer", _find_cartographer_launch)

    def start_cartographer_localization(self, state_file, mode="nonfix"):
        """Cartographer Localization 모드로 기동한다.

        Args:
            state_file: SLAM으로 생성된 .pbstream 파일 경로
            mode: "fix"   — AMCL처럼 고정된 맵에서 위치 추정만 수행 (/map 갱신 없음)
                  "nonfix" — 서브맵을 생성하며 /map 업데이트 + pose 보정 (기본값)
        """
        if not os.path.isfile(state_file):
            rospy.logerr(".pbstream 파일을 찾을 수 없습니다: %s", state_file)
            rospy.logerr("  SLAM으로 맵을 먼저 생성하세요:")
            rospy.logerr("    1. roslaunch woosh_slam_cartographer cartographer.launch")
            rospy.logerr("    2. roslaunch woosh_slam_cartographer save_state.launch map_name:=my_map")
            return False

        launch_file = _find_cartographer_localization_launch()
        if launch_file is None:
            rospy.logwarn("cartographer_localization.launch 파일을 찾을 수 없습니다.")
            return False

        localization_mode = "fix" if mode == "fix" else "nonfix"
        cmd = [
            "roslaunch", launch_file,
            f"robot_ip:={self.robot_ip}",
            f"robot_port:={self.robot_port}",
            f"state_file:={state_file}",
            f"localization_mode:={localization_mode}",
            "launch_rviz:=true",
            "launch_sensor_bridge:=false",
        ]

        stack_label = "Cartographer Localization (fix)" if mode == "fix" else "Cartographer Localization (nonfix)"
        started = self._pm.start(f"{stack_label} 스택", cmd)
        if started:
            rospy.loginfo("Cartographer Localization 시작 [mode=%s] (state_file=%s)", mode, state_file)
        return started

    # -- move_base --

    def start_move_base(
        self,
        global_planner_plugin="",
        local_planner_plugin="",
        move_base_params_file="",
        costmap_common_params_file="",
        global_costmap_params_file="",
        local_costmap_params_file="",
        global_planner_params_file="",
        local_planner_params_file="",
        load_global_planner_params=True,
        load_local_planner_params=True,
    ):
        """move_base 스택을 기동한다 (global/local costmap + 지정 플래너).

        sensor_bridge, map_server 또는 SLAM, localization이 이미 실행 중이어야 한다.
        플래너/YAML 인자가 비어 있으면 move_base_only.launch 기본값이 사용된다.
        """
        launch_file = _find_move_base_only_launch()
        if launch_file is None:
            rospy.logwarn("move_base_only.launch 파일을 찾을 수 없습니다. move_base를 시작하지 않습니다.")
            return False

        cmd = ["roslaunch", launch_file]

        if global_planner_plugin:
            cmd.append(f"global_planner_plugin:={global_planner_plugin}")
        if local_planner_plugin:
            cmd.append(f"local_planner_plugin:={local_planner_plugin}")
        if move_base_params_file:
            cmd.append(f"move_base_params_file:={move_base_params_file}")
        if costmap_common_params_file:
            cmd.append(f"costmap_common_params_file:={costmap_common_params_file}")
        if global_costmap_params_file:
            cmd.append(f"global_costmap_params_file:={global_costmap_params_file}")
        if local_costmap_params_file:
            cmd.append(f"local_costmap_params_file:={local_costmap_params_file}")
        if global_planner_params_file:
            cmd.append(f"global_planner_params_file:={global_planner_params_file}")
        if local_planner_params_file:
            cmd.append(f"local_planner_params_file:={local_planner_params_file}")

        cmd.append(f"load_global_planner_params:={'true' if load_global_planner_params else 'false'}")
        cmd.append(f"load_local_planner_params:={'true' if load_local_planner_params else 'false'}")

        started = self._pm.start("move_base 스택", cmd, startup_grace_sec=2.0)
        if started:
            rospy.loginfo(
                "move_base 스택 시작 (global=%s, local=%s)",
                global_planner_plugin or "navfn/NavfnROS(기본)",
                local_planner_plugin or "dwa_local_planner/DWAPlannerROS(기본)",
            )
        return started

# ---------------------------------------------------------------------------
# Navigation 명령 속도 CSV 로거
# ---------------------------------------------------------------------------

class NavCsvLogger:
    """navigation 구동 중 실시간 명령 속도·회전 방향을 CSV 파일로 기록한다.

    컬럼:
        timestamp      - Unix 절대 시각 (초, 소수점 4자리)
        elapsed_sec    - 로거 시작부터의 경과 시간 (초)
        source         - 명령 출처: "quintic" (직선 이동) | "cmd_vel" (move_base 패스스루)
        linear_m_s     - 선속도 (m/s, 양수=전진, 음수=후진)
        angular_rad_s  - 각속도 (rad/s, 양수=좌회전, 음수=우회전)
        direction      - forward | backward | rotate_ccw | rotate_cw | stop
        odom_x         - /odom 기준 현재 x 위치 (m, 없으면 빈 값)
        odom_y         - /odom 기준 현재 y 위치 (m, 없으면 빈 값)
    """

    FIELDNAMES = [
        "timestamp", "elapsed_sec",
        "source", "linear_m_s", "angular_rad_s",
        "direction", "odom_x", "odom_y",
    ]
    _LIN_THRESH = 0.001  # m/s — 정지 판정 임계값
    _ANG_THRESH = 0.001  # rad/s

    def __init__(self, log_dir=None):
        if log_dir is None:
            log_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))
        os.makedirs(log_dir, exist_ok=True)

        ts = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(log_dir, f"nav_cmd_{ts}.csv")

        # buffering=1: 라인 단위 즉시 flush → 노드 강제 종료 시에도 데이터 보존
        self._file = open(self._log_path, "w", newline="", buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._lock = Lock()
        self._start_time = time.monotonic()
        rospy.loginfo("[NavCsvLogger] 명령 속도 로그 시작: %s", self._log_path)

    @staticmethod
    def _direction_label(linear, angular):
        lin_moving = abs(linear) >= NavCsvLogger._LIN_THRESH
        ang_moving = abs(angular) >= NavCsvLogger._ANG_THRESH
        if not lin_moving and not ang_moving:
            return "stop"
        if lin_moving:
            return "forward" if linear > 0 else "backward"
        return "rotate_ccw" if angular > 0 else "rotate_cw"

    def log(self, source, linear, angular, odom_x=None, odom_y=None):
        """명령 속도 한 샘플을 CSV에 기록한다 (thread-safe)."""
        now = time.time()
        elapsed = time.monotonic() - self._start_time
        row = {
            "timestamp":      f"{now:.4f}",
            "elapsed_sec":    f"{elapsed:.4f}",
            "source":         source,
            "linear_m_s":     f"{linear:.6f}",
            "angular_rad_s":  f"{angular:.6f}",
            "direction":      self._direction_label(linear, angular),
            "odom_x":         f"{odom_x:.4f}" if odom_x is not None else "",
            "odom_y":         f"{odom_y:.4f}" if odom_y is not None else "",
        }
        with self._lock:
            self._writer.writerow(row)

    def close(self):
        with self._lock:
            try:
                self._file.close()
            except Exception:
                pass
        rospy.loginfo("[NavCsvLogger] 로그 저장 완료: %s", self._log_path)


# ---------------------------------------------------------------------------
# 모션 프로파일 (모바일 로봇의 부드러운 가감속)
# ---------------------------------------------------------------------------

def quintic_minimum_jerk_profile(tau):
    """Minimum Jerk 5차 다항식 속도 프로파일.

    Returns:
        (position_ratio, velocity_ratio, acceleration_ratio)
    """
    tau = np.clip(tau, 0.0, 1.0)
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    position_ratio = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    velocity_ratio = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
    acceleration_ratio = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3

    return position_ratio, velocity_ratio, acceleration_ratio


# ---------------------------------------------------------------------------
# SmoothTwistController
# ---------------------------------------------------------------------------

class SmoothTwistController:
    # Quintic 프로파일의 최대 속도 계수 (tau=0.5)
    QUINTIC_PEAK_VELOCITY_RATIO = 1.875
    OWNER_NAME = "SmoothTwistController"
    CALLER_NAME = "woosh_service_driver.py:SmoothTwistController"

    def __init__(self):
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_ctrl')
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.laser_frame = rospy.get_param("~laser_frame", "laser")
        self.publish_hz = max(1.0, float(rospy.get_param("~publish_hz", 10.0)))
        self.state_poll_sec = max(0.1, float(rospy.get_param("~state_poll_sec", 0.1)))
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.robot = None
        self._owner_record = None

        # 제어 파라미터
        self.max_speed = 0.12
        self.accel = 0.25
        self.control_hz = 50

        # startup / readiness
        self.startup_complete_event = Event()
        self.sdk_ready_event = Event()
        self.map_ready_event = Event()
        self.startup_error = None

        # 상태
        self.target_distance = 0.0
        self.estimated_distance = 0.0
        self.current_speed = 0.0
        self.is_moving = False

        self.command_queue = Queue()
        self.result_queue = Queue()

        # 맵 선택 결과 (carto_loc 연동용)
        self.selected_map_name = None   # 선택된 맵 이름 (확장자 없음)
        self.selected_map_source = None # "robot" 또는 "local"

        # odom / scan 상태 (SDK owner가 직접 발행)
        self._odom_lock = Lock()
        self._odom_pose = None
        self.latest_scan = None
        self.latest_pose_speed = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_twist_time = None
        self._sdk_pose_initialized = False
        self._pose_origin_x = 0.0
        self._pose_origin_y = 0.0
        self._pose_origin_theta = 0.0
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.odom_raw_pub = rospy.Publisher("/odom_raw", Odometry, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # cmd_vel 패스스루 (move_base_on 모드) — 별도 WebSocket 연결 없이 기존 연결 재사용
        self._cmd_vel_enabled = False
        self._cmd_vel_queue = Queue(maxsize=1)
        self._cmd_vel_lock = Lock()
        self._cmd_vel_last_time = None
        self._cmd_vel_last_linear = 0.0
        self._cmd_vel_last_angular = 0.0
        self._cmd_vel_watchdog_timeout = 1.0
        self._cmd_vel_sub = None   # enable_cmd_vel_passthrough() 호출 시 생성

        # dedicated latest-wins twist sender
        self._twist_queue = Queue(maxsize=1)
        self._shutdown_started = False

        # 명령 속도 CSV 로거
        self._csv_logger = NavCsvLogger()

    def _update_odom_pose_cache(self):
        with self._odom_lock:
            self._odom_pose = (self.odom_x, self.odom_y)

    # -- 센서 상태 --

    def _on_scan(self, scan):
        self.latest_scan = scan

    def _on_pose(self, pose_speed):
        self._update_odom_from_sdk_pose(pose_speed)
        self.latest_pose_speed = pose_speed

    def _update_odom_from_sdk_pose(self, pose_speed):
        pose = pose_speed.pose
        has_valid_pose = not (pose.x == 0.0 and pose.y == 0.0 and pose.theta == 0.0)

        if not has_valid_pose:
            self._integrate_twist_fallback(pose_speed)
            return

        if not self._sdk_pose_initialized:
            self._pose_origin_x = pose.x
            self._pose_origin_y = pose.y
            self._pose_origin_theta = pose.theta
            self._sdk_pose_initialized = True
            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_theta = 0.0
            self._update_odom_pose_cache()
            rospy.loginfo(
                "SDK pose 원점 설정: x=%.3f y=%.3f theta=%.3f (rad)",
                pose.x, pose.y, pose.theta,
            )
            return

        dx_world = pose.x - self._pose_origin_x
        dy_world = pose.y - self._pose_origin_y
        cos_o = math.cos(self._pose_origin_theta)
        sin_o = math.sin(self._pose_origin_theta)

        self.odom_x = dx_world * cos_o + dy_world * sin_o
        self.odom_y = -dx_world * sin_o + dy_world * cos_o
        self.odom_theta = math.atan2(
            math.sin(pose.theta - self._pose_origin_theta),
            math.cos(pose.theta - self._pose_origin_theta),
        )
        self._update_odom_pose_cache()

    def _integrate_twist_fallback(self, pose_speed):
        now = rospy.get_time()
        if self.last_twist_time is None:
            self.last_twist_time = now
            return

        dt = now - self.last_twist_time
        self.last_twist_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        linear = pose_speed.twist.linear
        angular = pose_speed.twist.angular
        self.odom_x += linear * math.cos(self.odom_theta) * dt
        self.odom_y += linear * math.sin(self.odom_theta) * dt
        self.odom_theta += angular * dt
        self.odom_theta = math.atan2(math.sin(self.odom_theta), math.cos(self.odom_theta))
        self._update_odom_pose_cache()

    async def _request_pose_once(self):
        try:
            pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok and pose_speed:
                self._update_odom_from_sdk_pose(pose_speed)
                self.latest_pose_speed = pose_speed
        except Exception as exc:
            rospy.logwarn("[SmoothTwistController] PoseSpeed 요청 실패: %s", exc)

    async def _request_scan_once(self):
        try:
            scan, ok, _ = await self.robot.scanner_data_req(ScannerData(), NO_PRINT, NO_PRINT)
            if ok and scan:
                self.latest_scan = scan
        except Exception as exc:
            rospy.logwarn("[SmoothTwistController] ScannerData 요청 실패: %s", exc)

    async def _setup_sensor_stream(self, info):
        if info.pose_speed:
            self.latest_pose_speed = info.pose_speed
            self.last_twist_time = rospy.get_time()
            self._update_odom_from_sdk_pose(info.pose_speed)

        try:
            ok = await self.robot.robot_pose_speed_sub(self._on_pose, NO_PRINT)
            if not ok:
                rospy.logwarn("[SmoothTwistController] PoseSpeed 구독 등록 실패 — 폴링 폴백 사용")
        except Exception as exc:
            rospy.logwarn("[SmoothTwistController] PoseSpeed 구독 예외: %s", exc)

        try:
            ok = await self.robot.scanner_data_sub(self._on_scan, NO_PRINT)
            if not ok:
                rospy.logwarn("[SmoothTwistController] ScannerData 구독 등록 실패 — 폴링 폴백 사용")
        except Exception as exc:
            rospy.logwarn("[SmoothTwistController] ScannerData 구독 예외: %s", exc)

        await self._request_pose_once()
        await self._request_scan_once()
        self.sdk_ready_event.set()

    def _publish_scan(self):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = scan.angle_min
        msg.angle_max = scan.angle_max
        msg.angle_increment = scan.angle_increment
        msg.time_increment = scan.time_increment
        msg.scan_time = scan.scan_time
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        msg.ranges = list(scan.ranges)
        self.scan_pub.publish(msg)

    def _build_odom_msg(self):
        now = rospy.Time.now()
        quat = _yaw_to_quaternion(self.odom_theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        if self.latest_pose_speed is not None:
            odom.twist.twist.linear.x = self.latest_pose_speed.twist.linear
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.latest_pose_speed.twist.angular
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.005

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.odom_x
            tf_msg.transform.translation.y = self.odom_y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf_msg)

        return odom

    def _publish_odom_and_tf(self):
        odom = self._build_odom_msg()
        self.odom_raw_pub.publish(odom)
        self.odom_pub.publish(odom)

    async def _sensor_publish_loop(self):
        rate = 1.0 / self.publish_hz
        last_poll = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - last_poll >= self.state_poll_sec:
                last_poll = now
                await self._request_pose_once()
                await self._request_scan_once()

            self._publish_scan()
            self._publish_odom_and_tf()
            await asyncio.sleep(rate)

    # -- 연결 / 초기화 --

    async def connect(self):
        conflicts = find_foreign_sdk_owners(
            self.robot_port,
            target_ip=self.robot_ip,
            ignore_pids={os.getpid()},
        )
        if conflicts:
            summary = ", ".join(
                f"pid={owner['pid']} proc={owner['proc']}" for owner in conflicts
            )
            for owner in conflicts:
                rospy.logerr(
                    "[SDK_OWNER] preflight_conflict pid=%s proc=%s raw=%s",
                    owner["pid"],
                    owner["proc"],
                    owner["line"],
                )
            log_sdk_owner(
                rospy.logerr,
                "preflight_conflict",
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
                note=summary,
            )
            raise RuntimeError(
                f"기존 SDK owner 연결이 이미 존재합니다. 먼저 종료하세요: {summary}"
            )

        log_sdk_owner(
            rospy.loginfo,
            "open_start",
            self.OWNER_NAME,
            self.robot_identity,
            self.robot_ip,
            self.robot_port,
            self.CALLER_NAME,
        )
        try:
            settings = CommuSettings(addr=self.robot_ip, port=self.robot_port, identity=self.robot_identity)
            self.robot = WooshRobot(settings)
            ok = await self.robot.run()
            if ok is False:
                raise RuntimeError("Woosh SDK 연결 시작 실패")

            self._owner_record = register_sdk_owner(
                rospy,
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
            )
            log_sdk_owner(
                rospy.loginfo,
                "open_established",
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
                note="single_owner_registered",
            )

            info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
            if not ok:
                raise RuntimeError("로봇 연결 실패")

            print_battery_status(info.battery.power)
            await self._setup_sensor_stream(info)
            rospy.loginfo("로봇 연결 성공! SDK single owner 준비 완료.")
            await self._setup_map()
            self.startup_complete_event.set()
        except Exception as exc:
            self.startup_error = str(exc)
            self.startup_complete_event.set()
            log_sdk_owner(
                rospy.logerr,
                "open_failed",
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
                note=str(exc),
            )
            raise

    async def _setup_map(self):
        """네비게이션 설정: 맵 로드 → 로컬라이제이션 → 초기화 → 자동 모드."""
        rospy.loginfo("=== 네비게이션 설정 시작 ===")

        map_loaded = await self._ensure_map_loaded()
        if map_loaded:
            await self._set_robot_pose()

        await self._init_robot()
        await self._switch_to_auto_mode()
        await self._log_final_state()

        rospy.loginfo("=== 네비게이션 설정 완료 ===")
        self.map_ready_event.set()

    async def _ensure_map_loaded(self):
        """현재 맵 상태를 확인하고 필요하면 맵을 로드한다.

        로봇 하드웨어 내부 맵(scene_list)과 프로젝트 로컬 맵(woosh_slam/maps)을
        합산한 목록에서 인덱스 2를 선택한다.
        - 로봇 내부 맵: switch_map_req로 로드
        - 로컬 맵: 파일 경로를 확인하고 로봇에 scene_name으로 등록 시도
        """
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logwarn("위치 정보 요청 실패: %s", msg)
            return False

        current_map_id = getattr(pose_speed, 'map_id', 0)
        if current_map_id != 0:
            rospy.loginfo("현재 로드된 맵 ID: %s", current_map_id)
            return True

        rospy.loginfo("현재 로드된 맵이 없습니다.")

        # ── 1. 로봇 하드웨어 내부 맵 목록 ──────────────────────────────────
        robot_scenes = []
        scene_list, ok, msg = await self.robot.scene_list_req(SceneList(), NO_PRINT, NO_PRINT)
        if ok and scene_list and scene_list.scenes:
            robot_scenes = [s.name for s in scene_list.scenes]

        # ── 2. 프로젝트 로컬 맵 목록 (woosh_slam/maps/*.yaml) ───────────────
        local_maps = _get_local_map_names()  # [{"name": str, "path": str}, ...]

        # ── 3. 통합 목록 구성: 로봇 내부 → 로컬 순서 ───────────────────────
        # 각 항목: {"name": str, "source": "robot"|"local", "path": str|None}
        combined = [{"name": n, "source": "robot", "path": None} for n in robot_scenes]
        combined += [{"name": m["name"], "source": "local", "path": m["path"]} for m in local_maps]

        if not combined:
            rospy.logwarn("사용 가능한 맵이 없습니다. (로봇 내부 및 로컬 모두 없음)")
            return False

        rospy.loginfo("── 로봇 내부 맵 (%d개) ──────────────────────────", len(robot_scenes))
        if robot_scenes:
            for i, name in enumerate(robot_scenes):
                rospy.loginfo("  [%d] %s", i, name)
        else:
            rospy.loginfo("  (없음)")

        rospy.loginfo("── 로컬 맵 (woosh_slam/maps, %d개) ─────────────", len(local_maps))
        if local_maps:
            for i, m in enumerate(local_maps):
                rospy.loginfo("  [%d] %s  (%s)", i, m["name"], m["path"])
        else:
            rospy.loginfo("  (없음)")

        rospy.loginfo("── 통합 맵 목록 (%d개) ──────────────────────────", len(combined))
        for i, e in enumerate(combined):
            rospy.loginfo("  [%d] %-30s  출처: %s", i, e["name"], e["source"])

        # ── 4. 인덱스 선택  ──────────────────────────────────────────────
        TARGET_INDEX = 4
        if len(combined) <= TARGET_INDEX:
            rospy.logwarn("통합 맵이 %d개 미만입니다. 인덱스 %d를 선택할 수 없습니다.",
                          TARGET_INDEX + 1, TARGET_INDEX)
            return False

        target = combined[TARGET_INDEX]
        rospy.loginfo("맵 로드 시도: %s (출처: %s)", target["name"], target["source"])

        # ── 5. 출처에 따라 로드 방식 결정 ───────────────────────────────────
        if target["source"] == "robot":
            switch_map = SwitchMap()
            switch_map.scene_name = target["name"]
            _, ok, msg = await self.robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
            if not ok:
                rospy.logerr("맵 로드 실패: %s", msg)
                return False
            rospy.loginfo("로봇 내부 맵 '%s' 로드 요청 성공", target["name"])

        else:  # local
            # 로컬 맵은 로봇 내부 API(switch_map_req)로 로드할 수 없다.
            # switch_map_req는 로봇 하드웨어에 저장된 scene_name만 인식하므로
            # 파일 유효성만 확인하고 AMCL용으로 활용한다.
            if not _validate_map_file(target["path"]):
                rospy.logerr("로컬 맵 파일 유효성 검사 실패: %s", target["path"])
                return False
            rospy.loginfo("로컬 맵 '%s' 선택됨 — AMCL & Cartographer 모드에서 활용 가능: %s",
                          target["name"], target["path"])

        # ── 6. 선택된 맵 정보 저장 (carto_loc 등 후속 스택에서 참조) ──────
        self.selected_map_name = target["name"]
        self.selected_map_source = target["source"]
        # 후속 localization 스택이 참조할 수 있도록 ROS 파라미터로 공유
        rospy.set_param("/woosh/selected_map_name", target["name"])
        rospy.set_param("/woosh/selected_map_source", target["source"])

        return True

    async def _set_robot_pose(self):
        """현재 위치를 맵 상의 위치로 설정(로컬라이제이션)."""
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return

        set_pose = SetRobotPose()
        set_pose.pose.x = pose_speed.pose.x
        set_pose.pose.y = pose_speed.pose.y
        set_pose.pose.theta = pose_speed.pose.theta

        _, ok, msg = await self.robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo("로봇 위치 설정 성공: (%.2f, %.2f, %.2f)",
                          set_pose.pose.x, set_pose.pose.y, set_pose.pose.theta)
            await asyncio.sleep(2)
        else:
            rospy.logwarn("로봇 위치 설정 실패: %s", msg)

    async def _init_robot(self):
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return

        init_robot = InitRobot()
        init_robot.is_record = False
        init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
        init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
        init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0

        _, ok, msg = await self.robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo("로봇 초기화 성공: (%.2f, %.2f, %.2f)",
                          init_robot.pose.x, init_robot.pose.y, init_robot.pose.theta)
            await asyncio.sleep(2)
        else:
            rospy.logwarn("로봇 초기화 실패: %s", msg)

    async def _switch_to_auto_mode(self):
        switch_mode = SwitchControlMode()
        switch_mode.mode = ControlMode.kAuto
        _, ok, msg = await self.robot.switch_control_mode_req(switch_mode, NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo("자동 제어 모드 설정 성공")
            await asyncio.sleep(2)
        else:
            rospy.logwarn("제어 모드 설정 실패: %s", msg)

    async def _log_final_state(self):
        state, ok, _ = await self.robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if not ok:
            return

        rospy.loginfo("[상태] robot=%s, nav=%s", bin(state.robot), bin(state.nav))

        if state.robot & OperationState.RobotBit.kTaskable:
            rospy.loginfo("로봇이 작업을 받을 수 있는 상태입니다.")
        else:
            rospy.loginfo("로봇이 아직 Taskable 상태가 아니지만, 작업 수행은 가능할 수 있습니다.")

        if state.nav & OperationState.NavBit.kImpede:
            rospy.logwarn("장애물이 감지되었습니다.")
        else:
            rospy.loginfo("네비게이션 경로가 깨끗합니다.")

    # -- 모션 --

    def _calculate_motion_time(self, distance):
        abs_distance = abs(distance)
        total_time = self.QUINTIC_PEAK_VELOCITY_RATIO * abs_distance / self.max_speed
        return np.clip(total_time, 0.3, 30.0)

    def _log_cmd(self, source, linear, angular):
        """현재 odom 위치와 함께 명령 속도를 CSV 로거에 기록한다."""
        with self._odom_lock:
            odom = self._odom_pose
        odom_x = odom[0] if odom is not None else None
        odom_y = odom[1] if odom is not None else None
        self._csv_logger.log(source, linear, angular, odom_x, odom_y)

    def _enqueue_twist(self, linear=0.0, angular=0.0, _source="quintic"):
        self._log_cmd(_source, linear, angular)
        try:
            self._twist_queue.get_nowait()
        except Empty:
            pass
        try:
            self._twist_queue.put_nowait((linear, angular, _source))
        except Exception:
            pass

    async def _twist_sender_loop(self):
        while not rospy.is_shutdown():
            try:
                linear, angular, _source = self._twist_queue.get_nowait()
            except Empty:
                await asyncio.sleep(0.005)
                continue

            try:
                await self.robot.twist_req(
                    Twist(linear=linear, angular=angular),
                    NO_PRINT,
                    NO_PRINT,
                )
            except Exception as exc:
                rospy.logwarn("[SmoothTwistController] twist_req 실패 (%s): %s", _source, exc)

    async def _flush_stop_commands(self, repeats=3, delay=0.05):
        if self.robot is None:
            return
        for _ in range(repeats):
            try:
                await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            except Exception as exc:
                rospy.logwarn("[SmoothTwistController] 종료 stop flush 실패: %s", exc)
            await asyncio.sleep(delay)

    # -- cmd_vel 패스스루 (move_base_on 모드) --

    def _cmd_vel_callback(self, msg):
        """move_base가 발행하는 /cmd_vel 을 수신해 큐에 넣는다.

        ROS 콜백 스레드에서 호출된다. asyncio 루프는 큐를 폴링한다.
        """
        linear = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
        angular = max(-0.5, min(0.5, float(msg.angular.z)))
        with self._cmd_vel_lock:
            self._cmd_vel_last_time = time.monotonic()
            self._cmd_vel_last_linear = linear
            self._cmd_vel_last_angular = angular
        # 큐가 꽉 찼으면 기존 값 버리고 최신으로 교체
        try:
            self._cmd_vel_queue.get_nowait()
        except Empty:
            pass
        try:
            self._cmd_vel_queue.put_nowait((linear, angular))
        except Exception:
            pass

    def enable_cmd_vel_passthrough(self, watchdog_timeout=1.0):
        """cmd_vel 패스스루 모드를 활성화한다 (move_base_on 전용).

        cmd_vel_adapter 서브프로세스 대신 기존 WebSocket 연결을 재사용하여
        SmoothTwistController + CmdVelAdapter 동시 연결 충돌을 방지한다.
        """
        if not self.sdk_ready_event.is_set() or self.robot is None:
            rospy.logerr(
                "[SmoothTwistController] cmd_vel 패스스루 활성화 실패 — SDK owner가 아직 준비되지 않았습니다."
            )
            raise RuntimeError("SDK owner not ready")

        self._cmd_vel_watchdog_timeout = watchdog_timeout
        if self._cmd_vel_sub is None:
            self._cmd_vel_sub = rospy.Subscriber(
                "/cmd_vel", RosTwist, self._cmd_vel_callback, queue_size=1
            )
        self._cmd_vel_enabled = True
        rospy.loginfo(
            "[SmoothTwistController] cmd_vel 패스스루 활성화 "
            "[owner=%s pid=%s identity=%s target=%s:%s watchdog=%.1fs]",
            self.OWNER_NAME,
            os.getpid(),
            self.robot_identity,
            self.robot_ip,
            self.robot_port,
            watchdog_timeout,
        )
        log_sdk_owner(
            rospy.loginfo,
            "passthrough_enabled",
            self.OWNER_NAME,
            self.robot_identity,
            self.robot_ip,
            self.robot_port,
            self.CALLER_NAME,
            note="cmd_vel_passthrough",
        )

    def run_move_base_self_check(self):
        rospy.loginfo("[SmoothTwistController] move_base_on self-check 시작")
        errors = []

        owner_ok, owner_record = current_process_is_registered_owner(rospy)
        if not owner_ok:
            errors.append(f"현재 프로세스가 등록된 SDK owner가 아닙니다: {owner_record}")

        if not self.sdk_ready_event.is_set() or self.robot is None:
            errors.append("SDK owner 연결이 아직 준비되지 않았습니다.")

        if not self._cmd_vel_enabled or self._cmd_vel_sub is None:
            errors.append("cmd_vel 패스스루가 활성화되지 않았습니다.")

        try:
            lines = inspect_tcp_connections(self.robot_port, target_ip=self.robot_ip)
            owners = parse_connection_owners(lines)
            for owner in owners:
                rospy.loginfo("[SDK_OWNER] observed_connection pid=%s proc=%s raw=%s",
                              owner["pid"], owner["proc"], owner["line"])
            if len(lines) != 1:
                errors.append(f"TCP 연결 수가 1이 아닙니다: {len(lines)}")
            elif owners and owners[0]["pid"] != os.getpid():
                errors.append(
                    f"유일 연결 owner가 현재 프로세스가 아닙니다: pid={owners[0]['pid']} proc={owners[0]['proc']}"
                )
        except Exception as exc:
            errors.append(f"`ss -tnp` self-check 실패: {exc}")

        if errors:
            for error in errors:
                rospy.logerr("[SmoothTwistController] move_base_on self-check 실패: %s", error)
            return False

        rospy.loginfo("[SmoothTwistController] move_base_on self-check 통과")
        return True

    async def _move_exact_distance(self, distance):
        if abs(distance) < 0.005:
            self._enqueue_twist()
            await asyncio.sleep(0.05)
            return True, f"완료: {distance:+.3f}m (너무 작음)"

        # 상태 초기화
        self.target_distance = distance
        self.estimated_distance = 0.0
        self.current_speed = 0.0
        self.is_moving = True

        period = 1.0 / self.control_hz
        direction = np.sign(distance)
        abs_distance = abs(distance)
        total_time = self._calculate_motion_time(abs_distance)

        # 이동 시작 시 odom 위치 스냅샷 (없으면 None → 속도 적분 폴백)
        with self._odom_lock:
            odom_start = self._odom_pose  # (x, y) or None

        if odom_start is not None:
            rospy.loginfo("[Quintic] odom 피드백 활성화 (시작 위치: x=%.3f, y=%.3f)",
                          odom_start[0], odom_start[1])
        else:
            rospy.logwarn("[Quintic] /odom 미수신 — 속도 적분 폴백으로 이동 거리 추정")

        rospy.loginfo("=" * 50)
        rospy.loginfo("[Quintic] 이동 시작: %+.3fm (%s), 예상 %.2fs",
                      distance, "전진" if direction > 0 else "후진", total_time)

        # 시간 기반 제어 루프
        loop = asyncio.get_event_loop()
        start_time = loop.time()
        last_time = start_time
        last_log_time = start_time
        cmd_integrated = 0.0  # 속도 적분 누적 (odom 불가 시 폴백용)

        while self.is_moving:
            now = loop.time()
            dt = max(now - last_time, period)
            last_time = now

            tau = min((now - start_time) / total_time, 1.0)
            _, vel_ratio, _ = quintic_minimum_jerk_profile(tau)

            target_speed = (abs_distance / total_time) * vel_ratio * direction
            target_speed = np.clip(target_speed, -self.max_speed, self.max_speed)

            # 급격한 속도 변화 방지 필터
            max_speed_change = self.accel * dt * 2.0
            speed_diff = target_speed - self.current_speed
            if abs(speed_diff) > max_speed_change:
                self.current_speed += np.sign(speed_diff) * max_speed_change
            else:
                self.current_speed = target_speed

            if tau >= 1.0:
                self.current_speed = 0.0
                self.is_moving = False

            self._enqueue_twist(self.current_speed)   # dedicated sender loop가 최신 명령만 전송
            cmd_integrated += self.current_speed * dt

            # 실제 이동 거리: odom 우선, 없으면 속도 적분 폴백
            with self._odom_lock:
                current_odom = self._odom_pose
            if odom_start is not None and current_odom is not None:
                dx = current_odom[0] - odom_start[0]
                dy = current_odom[1] - odom_start[1]
                self.estimated_distance = direction * math.sqrt(dx * dx + dy * dy)
            else:
                self.estimated_distance = cmd_integrated

            if now - last_log_time >= 0.5:
                src = "odom" if (odom_start is not None and current_odom is not None) else "추정"
                rospy.loginfo("[진행] tau=%.2f, v=%+.3fm/s, d=%+.3fm [%s]",
                              tau, self.current_speed, self.estimated_distance, src)
                last_log_time = now

            await asyncio.sleep(period)

        self.is_moving = False
        self._enqueue_twist(0.0, 0.0, _source="quintic_stop")
        await asyncio.sleep(period * 2.0)

        error = self.estimated_distance - distance
        src = "odom" if odom_start is not None else "추정"
        rospy.loginfo("[Quintic] 완료: 목표=%+.3fm, 실제=%+.3fm [%s], 오차=%.1fmm",
                      distance, self.estimated_distance, src, abs(error) * 1000)

        return True, f"완료: {self.estimated_distance:+.3f}m [%s] (오차: {error*1000:+.1f}mm)" % src

    # -- 메인 루프 --

    async def _control_loop(self):
        period_idle = 0.01          # /mobile_move 대기 루프 간격
        period_cmdvel = 1.0 / 20.0  # cmd_vel 패스스루 루프 간격 (20 Hz)
        _watchdog_fired = False

        while not rospy.is_shutdown():
            # ── /mobile_move 거리 명령 우선 처리 ────────────────────────────
            try:
                distance = self.command_queue.get_nowait()
                success, msg = await self._move_exact_distance(distance)
                self.result_queue.put((success, msg))
                continue
            except Empty:
                pass

            # ── cmd_vel 패스스루 (move_base_on 모드) ─────────────────────────
            if not self._cmd_vel_enabled:
                await asyncio.sleep(period_idle)
                continue

            linear, angular = 0.0, 0.0
            got_cmd = False
            try:
                linear, angular = self._cmd_vel_queue.get_nowait()
                got_cmd = True
            except Empty:
                pass

            with self._cmd_vel_lock:
                last_time = self._cmd_vel_last_time
                if not got_cmd:
                    linear = self._cmd_vel_last_linear
                    # hold-last angular 감쇠: DWA 발행 주기(200ms)의 2배(400ms) 초과 시
                    # angular를 0으로 폴백하여 stale 각속도 포화 구간 연장 방지.
                    # 250ms는 DWA 주기(200ms)와 너무 가까워 ROS 타이머 지터로 조기 발동 위험.
                    # 400ms(2× 주기)로 설정 시 정상 DWA 흐름에서 발동되지 않음.
                    if last_time is not None and (time.monotonic() - last_time) < 0.40:
                        angular = self._cmd_vel_last_angular
                    else:
                        angular = 0.0

            if last_time is None:
                # 아직 /cmd_vel 수신 없음 — 대기
                await asyncio.sleep(period_cmdvel)
                continue

            elapsed = time.monotonic() - last_time
            if elapsed >= self._cmd_vel_watchdog_timeout:
                if not _watchdog_fired:
                    rospy.logwarn(
                        "[SmoothTwistController] /cmd_vel %.1f초 미수신 — 자동 정지",
                        self._cmd_vel_watchdog_timeout,
                    )
                    _watchdog_fired = True
                linear, angular = 0.0, 0.0
            else:
                _watchdog_fired = False

            self._enqueue_twist(linear, angular, _source="cmd_vel")
            await asyncio.sleep(period_cmdvel)

    async def shutdown(self):
        if self._shutdown_started:
            return
        self._shutdown_started = True

        if self._cmd_vel_sub is not None:
            try:
                self._cmd_vel_sub.unregister()
            except Exception:
                pass
            self._cmd_vel_sub = None

        if self.robot is not None:
            log_sdk_owner(
                rospy.loginfo,
                "close_start",
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
            )
            await self._flush_stop_commands()
            try:
                await self.robot.stop()
            except Exception as exc:
                rospy.logwarn("[SmoothTwistController] SDK 종료 중 예외: %s", exc)
            log_sdk_owner(
                rospy.loginfo,
                "close_complete",
                self.OWNER_NAME,
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                self.CALLER_NAME,
            )
        clear_registered_sdk_owner(rospy)

    async def run(self):
        try:
            await self.connect()
            sensor_task = asyncio.create_task(self._sensor_publish_loop())
            twist_task = asyncio.create_task(self._twist_sender_loop())
            try:
                await self._control_loop()
            finally:
                sensor_task.cancel()
                twist_task.cancel()
                await asyncio.gather(sensor_task, twist_task, return_exceptions=True)
        finally:
            await self.shutdown()
            self._csv_logger.close()


# ---------------------------------------------------------------------------
# ROS 서비스 핸들러 / 메인
# ---------------------------------------------------------------------------

controller = None


def service_handler(req):
    if controller is None:
        return MoveMobileResponse(False, "서버 초기화 중")

    controller.command_queue.put(req.distance)
    try:
        success, msg = controller.result_queue.get(timeout=15.0)
        return MoveMobileResponse(success, msg)
    except Empty:
        return MoveMobileResponse(False, "타임아웃")


def _parse_cli_args(argv):
    """`rviz_on` / `amcl` / `gmap` / `carto_map` / `carto_loc_fix` / `carto_loc_nonfix` / `nav_on` 플래그를 분리하고 나머지는 ROS argv로 유지."""
    flags = {
        "rviz": False, "amcl": False, "gmap": False,
        "carto_map": False,
        "carto_loc_fix": False,    # 고정 맵 localization (AMCL 유사)
        "carto_loc_nonfix": False, # 서브맵 업데이트 포함 localization
        "nav_on": False,           # localization 연동 Global Costmap (costmap_2d standalone)
        "move_base_on": False,     # move_base 자율 내비게이션 (navfn + DWA + cmd_vel passthrough)
        "legacy_costmap": False,   # 구버전 별칭
    }
    map_file = None
    state_file = None
    filtered = [argv[0]]

    FLAG_MAP = {
        "rviz_on": "rviz", "--rviz_on": "rviz",
        "amcl": "amcl", "--amcl": "amcl",
        "gmap": "gmap", "--gmap": "gmap",
        "carto_map": "carto_map", "--carto_map": "carto_map",
        "carto_loc_fix": "carto_loc_fix", "--carto_loc_fix": "carto_loc_fix",
        "carto_loc_nonfix": "carto_loc_nonfix", "--carto_loc_nonfix": "carto_loc_nonfix",
        "nav_on": "nav_on", "--nav_on": "nav_on",
        "move_base_on": "move_base_on", "--move_base_on": "move_base_on",
        "costmap": "legacy_costmap", "--costmap": "legacy_costmap",
    }

    for arg in argv[1:]:
        key = arg.lower()
        if key in FLAG_MAP:
            flag_name = FLAG_MAP[key]
            flags[flag_name] = True
            if flag_name == "legacy_costmap":
                flags["nav_on"] = True
            if key in ("amcl", "--amcl"):
                flags["rviz"] = True
            continue
        if key.startswith("map_file:="):
            map_file = arg[len("map_file:="):]
            continue
        if key.startswith("state_file:="):
            state_file = arg[len("state_file:="):]
            continue
        filtered.append(arg)

    return flags, map_file, state_file, filtered


def _get_selected_localizations(flags):
    return [
        name for name in ("amcl", "carto_loc_fix", "carto_loc_nonfix")
        if flags[name]
    ]


def _wait_for_selected_map_name(timeout=30.0):
    start_time = time.monotonic()
    ctrl = None

    while (time.monotonic() - start_time) < min(timeout, 5.0):
        if controller is not None:
            ctrl = controller
            break
        if rospy.is_shutdown():
            return None
        time.sleep(0.05)

    if ctrl is None:
        rospy.logwarn("컨트롤러 초기화 대기 타임아웃. 기본 경로를 사용합니다.")
        return None

    remaining = max(timeout - (time.monotonic() - start_time), 0.0)
    rospy.loginfo("맵 선택 완료 대기 중... (최대 %.0f초)", remaining)
    if not ctrl.map_ready_event.wait(timeout=remaining):
        rospy.logwarn("맵 선택 대기 타임아웃. 기본 경로를 사용합니다.")
        return None

    return ctrl.selected_map_name


def _resolve_map_file(cli_map_file, purpose_label="맵"):
    if cli_map_file:
        return cli_map_file

    selected_map_name = _wait_for_selected_map_name()
    if selected_map_name:
        found = _find_yaml_for_map(selected_map_name)
        if found:
            rospy.loginfo("선택된 맵 '%s'에 대응하는 .yaml 파일 발견 (%s): %s",
                          selected_map_name, purpose_label, found)
            return found
        rospy.logwarn("선택된 맵 '%s'에 대응하는 .yaml 파일이 없습니다. 기본 경로 사용 (%s)",
                      selected_map_name, purpose_label)
    else:
        rospy.logwarn("선택된 맵을 확인하지 못했습니다. 기본 경로 사용 (%s)", purpose_label)

    return rospy.get_param("~map_file", DEFAULT_MAP_FILE)


def _resolve_state_file(cli_state_file):
    if cli_state_file:
        return cli_state_file

    selected_map_name = _wait_for_selected_map_name()
    if selected_map_name:
        found = _find_pbstream_for_map(selected_map_name)
        if found:
            rospy.loginfo("선택된 맵 '%s'에 대응하는 .pbstream 파일 발견: %s",
                          selected_map_name, found)
            return found
        rospy.logwarn("선택된 맵 '%s'에 대응하는 .pbstream 파일이 없습니다. 기본 경로 사용",
                      selected_map_name)
    else:
        rospy.logwarn("선택된 맵을 확인하지 못했습니다. 기본 state_file 경로 사용")

    return rospy.get_param("~state_file", DEFAULT_CARTO_STATE_FILE)


def _resolve_nav_map_file(localization_mode, cli_map_file, resolved_amcl_map=None, resolved_state_file=None):
    if localization_mode == "amcl":
        return resolved_amcl_map or _resolve_map_file(cli_map_file, purpose_label="nav_on")

    if cli_map_file:
        return cli_map_file

    if resolved_state_file:
        derived_map = _find_yaml_for_state_file(resolved_state_file)
        if derived_map:
            rospy.loginfo("선택된 state_file에 대응하는 .yaml 파일 발견 (nav_on): %s", derived_map)
            return derived_map

    if localization_mode == "carto_loc_fix":
        return _resolve_map_file(None, purpose_label="carto_loc_fix + nav_on")

    return rospy.get_param("~map_file", DEFAULT_MAP_FILE)


def _wait_for_controller_ready(timeout=25.0):
    start = time.monotonic()
    while controller is None and (time.monotonic() - start) < min(timeout, 5.0):
        if rospy.is_shutdown():
            return None
        time.sleep(0.05)

    if controller is None:
        rospy.logerr("SmoothTwistController 인스턴스를 만들지 못했습니다.")
        return None

    remaining = max(timeout - (time.monotonic() - start), 0.0)
    if not controller.startup_complete_event.wait(timeout=remaining):
        rospy.logerr("SmoothTwistController SDK owner 초기화 대기 타임아웃")
        return None

    if controller.startup_error:
        rospy.logerr("SmoothTwistController SDK owner 초기화 실패: %s", controller.startup_error)
        return None

    return controller


def _run_asyncio():
    global controller
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    controller = SmoothTwistController()
    try:
        loop.run_until_complete(controller.run())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if controller is not None:
            controller.startup_error = str(exc)
            controller.startup_complete_event.set()
        rospy.logerr("SmoothTwistController asyncio 루프 예외: %s", exc)
    finally:
        loop.close()


def _validate_modes(slam_mode, localization_mode, navigation_mode):
    """모드 조합 유효성 검사. 잘못된 조합이면 오류 메시지 출력 후 False 반환."""
    if slam_mode != "none" and localization_mode != "none":
        rospy.logerr(
            "SLAM(%s)과 localization(%s)을 동시에 실행할 수 없습니다.",
            slam_mode, localization_mode,
        )
        return False

    if navigation_mode != "none" and slam_mode == "none" and localization_mode == "none":
        rospy.logerr(
            "navigation_mode=%s를 사용하려면 slam_mode 또는 localization_mode가 필요합니다.",
            navigation_mode,
        )
        return False

    if localization_mode == "carto_nonfix":
        pass  # map_file / state_file 검증은 main()에서 처리

    return True


def main():
    # 1. CLI argv 파싱 (rospy.init_node 전에 수행 — ROS args 필터링 목적)
    flags, cli_map_file, cli_state_file, init_argv = _parse_cli_args(sys.argv)

    # 2. ROS 노드 초기화
    rospy.init_node('mobile_move_server', anonymous=False, argv=init_argv)

    # 3. ROS param에서 정식 모드 읽기 (woosh_navigation_system.launch가 설정)
    slam_mode         = rospy.get_param('~slam_mode',         'none').strip().lower()
    localization_mode = rospy.get_param('~localization_mode', 'none').strip().lower()
    navigation_mode   = rospy.get_param('~navigation_mode',   'none').strip().lower()
    launch_rviz       = rospy.get_param('~launch_rviz',       False)
    rviz_config_param = rospy.get_param('~rviz_config',       '')
    map_file          = rospy.get_param('~map_file',          cli_map_file or '')
    state_file        = rospy.get_param('~state_file',        cli_state_file or '')

    # 플래너/YAML 경로 params
    global_planner_plugin      = rospy.get_param('~global_planner_plugin',      'navfn/NavfnROS')
    local_planner_plugin       = rospy.get_param('~local_planner_plugin',       'dwa_local_planner/DWAPlannerROS')
    move_base_params_file      = rospy.get_param('~move_base_params_file',      '')
    costmap_common_params_file = rospy.get_param('~costmap_common_params_file', '')
    global_costmap_params_file = rospy.get_param('~global_costmap_params_file', '')
    local_costmap_params_file  = rospy.get_param('~local_costmap_params_file',  '')
    global_planner_params_file = rospy.get_param('~global_planner_params_file', '')
    local_planner_params_file  = rospy.get_param('~local_planner_params_file',  '')
    load_global_planner_params = rospy.get_param('~load_global_planner_params', True)
    load_local_planner_params  = rospy.get_param('~load_local_planner_params',  True)

    # 타임아웃 params
    nav_prerequisites_timeout = float(rospy.get_param('~nav_prerequisites_timeout', 30.0))
    costmap_ready_timeout     = float(rospy.get_param('~costmap_ready_timeout',     20.0))

    robot_ip   = rospy.get_param('~robot_ip',   '169.254.128.2')
    robot_port = rospy.get_param('~robot_port', 5480)

    # 4. CLI 플래그가 있으면 ROS param 값을 덮어씀 (레거시 별칭 번역)
    if flags.get('gmap'):              slam_mode         = 'gmapping'
    if flags.get('carto_map'):         slam_mode         = 'cartographer'
    if flags.get('amcl'):              localization_mode = 'amcl'
    if flags.get('carto_loc_fix'):     localization_mode = 'carto_fix'
    if flags.get('carto_loc_nonfix'):  localization_mode = 'carto_nonfix'
    if flags.get('nav_on') or flags.get('legacy_costmap'):
                                       navigation_mode   = 'costmap'
    if flags.get('move_base_on'):      navigation_mode   = 'move_base'
    if flags.get('rviz'):              launch_rviz       = True

    # CLI로 넘어온 파일 경로가 있으면 우선 적용
    if cli_map_file:
        map_file = cli_map_file
    if cli_state_file:
        state_file = cli_state_file

    if flags.get('legacy_costmap'):
        rospy.logwarn("`costmap` 플래그는 더 이상 권장되지 않습니다. `nav_on`을 사용하세요.")

    # 5. 유효성 검사
    if not _validate_modes(slam_mode, localization_mode, navigation_mode):
        return

    # carto_nonfix + navigation 조합은 map_file + state_file 모두 필요
    if localization_mode == 'carto_nonfix' and navigation_mode != 'none':
        if not map_file or not state_file:
            rospy.logerr(
                "carto_nonfix + navigation_mode=%s 조합은 map_file과 state_file 모두 필요합니다.",
                navigation_mode,
            )
            return

    launcher = StackLauncher(robot_ip, robot_port)
    rospy.on_shutdown(launcher.shutdown)
    atexit.register(launcher.shutdown)

    Thread(target=_run_asyncio, daemon=True).start()
    rospy.Service('mobile_move', MoveMobile, service_handler)
    ctrl = _wait_for_controller_ready(timeout=25.0)
    if ctrl is None:
        rospy.logerr("SDK owner 초기화에 실패하여 bringup을 중단합니다.")
        return

    # 6. 기본 모드 (SLAM/localization 없음): base_link→laser TF를 드라이버가 직접 발행
    if slam_mode == 'none' and localization_mode == 'none':
        launcher.start_base_laser_tf()

    # 7. RViz 시작 (SLAM/loc 스택이 자체적으로 RViz를 시작하지 않는 경우)
    slam_or_loc_active = slam_mode != 'none' or localization_mode != 'none'
    if launch_rviz and not slam_or_loc_active:
        launcher.start_rviz(
            use_amcl_rviz=False,
            require_nav_costmap=(navigation_mode != 'none'),
        )
    elif launch_rviz:
        rospy.loginfo("선택한 스택에서 RViz가 자동 실행되므로 추가 launch_rviz 요청은 건너뜁니다.")

    # 8. SLAM 스택 시작
    if slam_mode == 'gmapping':
        launcher.start_gmapping()
    elif slam_mode == 'cartographer':
        launcher.start_cartographer()

    # 9. Localization 스택 시작
    localization_started = False
    resolved_amcl_map = None
    resolved_state = None

    if localization_mode == 'amcl':
        resolved_amcl_map = _resolve_map_file(map_file or None, purpose_label="AMCL")
        localization_started = launcher.start_amcl(resolved_amcl_map)

    elif localization_mode in ('carto_fix', 'carto_nonfix'):
        # 레거시 CLI 호환: carto_loc_fix / carto_loc_nonfix → carto_fix / carto_nonfix
        carto_mode = 'fix' if localization_mode == 'carto_fix' else 'nonfix'
        resolved_state = _resolve_state_file(state_file or None)
        localization_started = launcher.start_cartographer_localization(resolved_state, mode=carto_mode)

    # 10. Navigation 스택 시작
    if navigation_mode != 'none':
        # SLAM 모드는 localization_started 없이 직접 진행
        source_started = localization_started if slam_mode == 'none' else True

        if not source_started:
            rospy.logerr("localization 스택이 정상적으로 시작되지 않아 nav 스택을 중단합니다.")
        else:
            nav_prereq_ok = launcher.wait_for_nav_prerequisites(
                localization_mode=localization_mode,
                slam_mode=slam_mode,
                timeout=nav_prerequisites_timeout,
            )
            if not nav_prereq_ok:
                rospy.logwarn("nav 필수 준비 신호가 완전히 확인되지 않았지만 기동을 시도합니다.")

            # SLAM 모드에서는 SLAM 노드가 /map을 직접 발행 → map_server 불필요
            launch_map_server_for_costmap = (
                localization_mode in ('carto_fix', 'carto_nonfix')
                and slam_mode == 'none'
            )

            if navigation_mode == 'move_base':
                rospy.loginfo(
                    "move_base 요청 감지 — slam=%s, localization=%s",
                    slam_mode, localization_mode,
                )
                try:
                    ctrl.enable_cmd_vel_passthrough()
                except RuntimeError as exc:
                    rospy.logerr("cmd_vel 패스스루 활성화 실패: %s", exc)
                    rospy.signal_shutdown("cmd_vel passthrough enable failed")
                    return
                if not ctrl.run_move_base_self_check():
                    rospy.logerr("move_base self-check 실패 — move_base 시작을 중단합니다.")
                    rospy.signal_shutdown("move_base self-check failed")
                    return
                launcher.start_move_base(
                    global_planner_plugin=global_planner_plugin,
                    local_planner_plugin=local_planner_plugin,
                    move_base_params_file=move_base_params_file,
                    costmap_common_params_file=costmap_common_params_file,
                    global_costmap_params_file=global_costmap_params_file,
                    local_costmap_params_file=local_costmap_params_file,
                    global_planner_params_file=global_planner_params_file,
                    local_planner_params_file=local_planner_params_file,
                    load_global_planner_params=load_global_planner_params,
                    load_local_planner_params=load_local_planner_params,
                )

            elif navigation_mode == 'costmap':
                # standalone Global Costmap (costmap_2d_node)
                costmap_map = _resolve_nav_map_file(
                    localization_mode, map_file or None,
                    resolved_amcl_map=resolved_amcl_map,
                    resolved_state_file=resolved_state,
                )
                rospy.loginfo(
                    "costmap 요청 감지 — slam=%s, localization=%s, map_server=%s",
                    slam_mode, localization_mode,
                    "on" if launch_map_server_for_costmap else "off",
                )
                costmap_started = launcher.start_costmap(
                    costmap_map,
                    launch_map_server=launch_map_server_for_costmap,
                    launch_map_odom_tf=False,
                    launch_base_laser_tf=False,
                )
                if costmap_started:
                    launcher.wait_for_costmap_ready(
                        localization_mode=localization_mode,
                        timeout=costmap_ready_timeout,
                    )

    rospy.loginfo("서버 시작됨! (Quintic Minimum Jerk 정밀 제어)")
    rospy.loginfo("  rosservice call /mobile_move \"{distance: 0.3}\"")
    rospy.loginfo("  정식 실행: roslaunch woosh_bringup woosh_navigation_system.launch [args]")
    rospy.loginfo("  레거시 CLI: rosrun woosh_bringup woosh_service_driver.py [flags]")
    rospy.spin()


if __name__ == "__main__":
    main()
