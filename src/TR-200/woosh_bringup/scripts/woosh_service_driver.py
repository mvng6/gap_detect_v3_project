#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import asyncio
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

# === Python 경로 설정 ===
script_dir = os.path.dirname(os.path.abspath(__file__))

woosh_robot_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_robot_py"))
if woosh_robot_dir not in sys.path:
    sys.path.insert(0, woosh_robot_dir)

try:
    from woosh_utils import print_battery_status
except ImportError:
    woosh_utils_src_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_utils/src"))
    if woosh_utils_src_dir not in sys.path:
        sys.path.insert(0, woosh_utils_src_dir)
    from woosh_utils import print_battery_status

from woosh_msgs.srv import MoveMobile, MoveMobileResponse
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
from woosh.proto.robot.robot_pack_pb2 import Twist, SwitchMap, SetRobotPose, InitRobot, SwitchControlMode
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed, OperationState
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.robot_pb2 import ControlMode


DEFAULT_MAP_FILE = "/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml"
DEFAULT_CARTO_STATE_FILE = "/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream"


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


def _find_debug_script():
    return _resolve_file(
        os.path.join(script_dir, "woosh_rviz_debug.py"),
        os.path.join(_find_package_dir(), "scripts", "woosh_rviz_debug.py"),
    )


def _find_sensor_bridge_script():
    return _resolve_file(
        os.path.abspath(os.path.join(script_dir, "../../woosh_sensor_bridge/scripts/woosh_sensor_bridge.py")),
        rospkg_fallback=("woosh_sensor_bridge", os.path.join("scripts", "woosh_sensor_bridge.py")),
    )


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

    # -- 센서 브릿지 --

    def start_sensor_bridge(self):
        bridge_script = _find_sensor_bridge_script()
        if bridge_script is None:
            rospy.logwarn("woosh_sensor_bridge.py를 찾을 수 없습니다. /scan이 발행되지 않습니다.")
            return False
        return self._pm.start("센서 브릿지", [
            sys.executable, bridge_script,
            f"_robot_ip:={self.robot_ip}",
            f"_robot_port:={self.robot_port}",
            "_robot_identity:=service_bridge",
            "_publish_hz:=10.0",
        ])

    def wait_for_nav_prerequisites(self, localization_mode, timeout=20.0):
        """nav_on 시작 전 필수 토픽/TF가 준비될 때까지 잠시 대기한다."""
        deadline = time.monotonic() + timeout
        all_ok = True

        def remaining_time():
            return max(deadline - time.monotonic(), 0.0)

        required_topics = [
            ("/scan", LaserScan, "LiDAR /scan"),
            ("/odom", Odometry, "Odometry /odom"),
        ]

        if localization_mode in ("amcl", "carto_loc_fix", "carto_loc_nonfix"):
            required_topics.append(("/map", OccupancyGrid, "Map /map"))

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

        if localization_mode == "amcl":
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

        elif localization_mode in ("carto_loc_fix", "carto_loc_nonfix"):
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

    def wait_for_costmap_ready(self, localization_mode=None, timeout=15.0):
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
        debug_script = _find_debug_script()
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

        if not debug_script:
            rospy.logwarn("`woosh_rviz_debug.py`를 찾지 못해 RViz 지원을 시작하지 않습니다.")
            return False
        if not rviz_config:
            rospy.logwarn("RViz 설정 파일(%s)을 찾지 못해 RViz 지원을 시작하지 않습니다.", config_label)
            return False
        if not rviz_bin:
            rospy.logwarn("`rviz` 실행 파일을 찾지 못했습니다. `rviz_on` 요청은 건너뜁니다.")
            return False

        debug_ok = self._pm.start("RViz 디버그 노드", [
            sys.executable, debug_script,
            f"_robot_ip:={self.robot_ip}",
            f"_robot_port:={self.robot_port}",
            "_robot_identity:=rviz_debug",
        ])
        rviz_ok = self._pm.start("RViz", [rviz_bin, "-d", rviz_config])
        if debug_ok and rviz_ok:
            rospy.loginfo("RViz 디버그 창 시작 (설정: %s)", config_label)
        return debug_ok and rviz_ok

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

    def __init__(self):
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_ctrl')

        self.robot = None

        # 제어 파라미터
        self.max_speed = 0.12
        self.accel = 0.25
        self.control_hz = 50

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
        self.map_ready_event = Event()  # 맵 선택 완료 시그널

    # -- 연결 / 초기화 --

    async def connect(self):
        settings = CommuSettings(addr=self.robot_ip, port=self.robot_port, identity=self.robot_identity)
        self.robot = WooshRobot(settings)
        await self.robot.run()

        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 연결 실패")

        print_battery_status(info.battery.power)
        rospy.loginfo("로봇 연결 성공!")
        await self._setup_map()

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
        # 센서 브릿지(서브프로세스)가 참조할 수 있도록 ROS 파라미터로도 공유
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

    async def _send_twist(self, linear=0.0):
        await self.robot.twist_req(Twist(linear=linear, angular=0.0), NO_PRINT, NO_PRINT)

    async def _move_exact_distance(self, distance):
        if abs(distance) < 0.005:
            await self._send_twist()
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

        rospy.loginfo("=" * 50)
        rospy.loginfo("[Quintic] 이동 시작: %+.3fm (%s), 예상 %.2fs",
                      distance, "전진" if direction > 0 else "후진", total_time)

        # 시간 기반 제어 루프
        loop = asyncio.get_event_loop()
        start_time = loop.time()
        last_time = start_time
        last_log_time = start_time

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

            await self._send_twist(self.current_speed)
            self.estimated_distance += self.current_speed * dt

            if now - last_log_time >= 0.5:
                rospy.loginfo("[진행] tau=%.2f, v=%+.3fm/s, d=%+.3fm",
                              tau, self.current_speed, self.estimated_distance)
                last_log_time = now

            await asyncio.sleep(period)

        # 정지 명령 반복
        for _ in range(5):
            await self._send_twist()
            await asyncio.sleep(period)

        self.is_moving = False

        error = self.estimated_distance - distance
        rospy.loginfo("[Quintic] 완료: 목표=%+.3fm, 추정=%+.3fm, 오차=%.1fmm",
                      distance, self.estimated_distance, abs(error) * 1000)

        return True, f"완료: {self.estimated_distance:+.3f}m (오차: {error*1000:+.1f}mm)"

    # -- 메인 루프 --

    async def _control_loop(self):
        while True:
            try:
                distance = self.command_queue.get_nowait()
            except Empty:
                await asyncio.sleep(0.01)
                continue

            success, msg = await self._move_exact_distance(distance)
            self.result_queue.put((success, msg))

    async def run(self):
        await self.connect()
        await self._control_loop()


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
        "nav_on": False,           # localization 연동 Global Costmap (costmap_2d)
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


def _run_asyncio():
    global controller
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    controller = SmoothTwistController()
    try:
        loop.run_until_complete(controller.run())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()


def main():
    flags, map_file, state_file, init_argv = _parse_cli_args(sys.argv)

    rospy.init_node('mobile_move_server', anonymous=False, argv=init_argv)

    robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
    robot_port = rospy.get_param("~robot_port", 5480)
    launcher = StackLauncher(robot_ip, robot_port)

    rospy.on_shutdown(launcher.shutdown)
    atexit.register(launcher.shutdown)

    selected_localizations = _get_selected_localizations(flags)
    if len(selected_localizations) > 1:
        rospy.logerr("localization 모드는 하나만 선택할 수 있습니다: amcl | carto_loc_fix | carto_loc_nonfix")
        return

    localization_mode = selected_localizations[0] if selected_localizations else None

    if flags["legacy_costmap"]:
        rospy.logwarn("`costmap` 플래그는 더 이상 권장되지 않습니다. `nav_on`을 사용하세요.")

    if flags["nav_on"] and localization_mode is None:
        rospy.logerr("`nav_on`은 localization 모드와 함께 사용해야 합니다: amcl | carto_loc_fix | carto_loc_nonfix")
        return

    if flags["nav_on"] and (flags["gmap"] or flags["carto_map"]):
        rospy.logerr("`nav_on`은 SLAM 모드(gmap, carto_map)와 함께 사용할 수 없습니다.")
        return

    Thread(target=_run_asyncio, daemon=True).start()
    rospy.Service('mobile_move', MoveMobile, service_handler)

    # 센서 브릿지는 항상 기동
    launcher.start_sensor_bridge()

    manual_rviz_modes = not (flags["gmap"] or flags["carto_map"] or flags["carto_loc_fix"] or flags["carto_loc_nonfix"])
    if flags["rviz"] and manual_rviz_modes:
        launcher.start_rviz(
            use_amcl_rviz=(localization_mode == "amcl"),
            require_nav_costmap=flags["nav_on"],
        )
    elif flags["rviz"]:
        rospy.loginfo("선택한 스택에서 RViz가 자동 실행되므로 추가 `rviz_on` 요청은 건너뜁니다.")

    resolved_amcl_map = None
    localization_started = False
    if flags["amcl"]:
        resolved_amcl_map = _resolve_map_file(map_file, purpose_label="AMCL")
        localization_started = launcher.start_amcl(resolved_amcl_map)

    if flags["gmap"]:
        launcher.start_gmapping()

    if flags["carto_map"]:
        launcher.start_cartographer()

    resolved_state = None
    _carto_loc_mode = None
    if localization_mode == "carto_loc_fix":
        _carto_loc_mode = "fix"
    elif localization_mode == "carto_loc_nonfix":
        _carto_loc_mode = "nonfix"

    if _carto_loc_mode is not None:
        resolved_state = _resolve_state_file(state_file)
        localization_started = launcher.start_cartographer_localization(resolved_state, mode=_carto_loc_mode)

    if flags["nav_on"]:
        if not localization_started:
            rospy.logerr("localization 스택이 정상적으로 시작되지 않아 `nav_on`을 중단합니다.")
        else:
            nav_prereq_ok = launcher.wait_for_nav_prerequisites(localization_mode)
            if not nav_prereq_ok:
                rospy.logwarn("nav_on 필수 준비 신호가 완전히 확인되지 않았지만 Global Costmap 기동을 시도합니다.")

            launch_map_server = (localization_mode == "carto_loc_fix")
            costmap_map = _resolve_nav_map_file(localization_mode, map_file,
                                                resolved_amcl_map=resolved_amcl_map,
                                                resolved_state_file=resolved_state)
            rospy.loginfo("nav_on 요청 감지 — localization=%s, costmap map_server=%s",
                          localization_mode, "on" if launch_map_server else "off")
            costmap_started = launcher.start_costmap(costmap_map, launch_map_server=launch_map_server,
                                                     launch_map_odom_tf=False,
                                                     launch_base_laser_tf=False)
            if costmap_started:
                launcher.wait_for_costmap_ready(localization_mode=localization_mode)

    rospy.loginfo("서버 시작됨! (Quintic Minimum Jerk 정밀 제어)")
    rospy.loginfo("  rosservice call /mobile_move \"{distance: 0.3}\"")
    rospy.loginfo("  옵션: rviz_on | amcl [map_file:=...] | gmap | carto_map")
    rospy.loginfo("         carto_loc_fix [state_file:=...]      — 고정 맵 localization (AMCL 유사)")
    rospy.loginfo("         carto_loc_nonfix [state_file:=...]   — 서브맵 업데이트 + pose 보정")
    rospy.loginfo("         nav_on [map_file:=...]               — 선택한 localization에 Global Costmap 적용")
    rospy.spin()


if __name__ == "__main__":
    main()
