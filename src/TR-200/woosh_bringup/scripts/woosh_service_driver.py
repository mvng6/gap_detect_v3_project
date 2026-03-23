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
from queue import Queue, Empty
from threading import Thread, Event

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


# ---------------------------------------------------------------------------
# 서브프로세스 관리
# ---------------------------------------------------------------------------

class SubprocessManager:
    """자식 프로세스의 생성/종료를 관리한다."""

    def __init__(self):
        self._procs = {}  # name -> Popen

    def start(self, name, cmd):
        try:
            self._procs[name] = subprocess.Popen(cmd, start_new_session=True)
            rospy.loginfo("%s 시작", name)
        except Exception as exc:
            rospy.logwarn("%s 시작 실패: %s", name, exc)

    def stop(self, name):
        proc = self._procs.pop(name, None)
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
            return
        self._pm.start("센서 브릿지", [
            sys.executable, bridge_script,
            f"_robot_ip:={self.robot_ip}",
            f"_robot_port:={self.robot_port}",
            "_robot_identity:=service_bridge",
            "_publish_hz:=10.0",
        ])

    # -- RViz --

    def start_rviz(self, use_amcl_rviz=False):
        debug_script = _find_debug_script()
        rviz_bin = _find_rviz_binary()

        if use_amcl_rviz:
            rviz_config = _find_amcl_rviz_config()
            config_label = "amcl_debug.rviz"
        else:
            rviz_config = _find_rviz_config()
            config_label = "woosh_rviz_debug.rviz"

        if not debug_script:
            rospy.logwarn("`woosh_rviz_debug.py`를 찾지 못해 RViz 지원을 시작하지 않습니다.")
            return
        if not rviz_config:
            rospy.logwarn("RViz 설정 파일(%s)을 찾지 못해 RViz 지원을 시작하지 않습니다.", config_label)
            return
        if not rviz_bin:
            rospy.logwarn("`rviz` 실행 파일을 찾지 못했습니다. `rviz_on` 요청은 건너뜁니다.")
            return

        self._pm.start("RViz 디버그 노드", [
            sys.executable, debug_script,
            f"_robot_ip:={self.robot_ip}",
            f"_robot_port:={self.robot_port}",
            "_robot_identity:=rviz_debug",
        ])
        self._pm.start("RViz", [rviz_bin, "-d", rviz_config])
        rospy.loginfo("RViz 디버그 창 시작 (설정: %s)", config_label)

    # -- AMCL --

    def start_amcl(self, map_file):
        if not _validate_map_file(map_file):
            rospy.logerr("AMCL을 시작하지 않습니다. 위 오류를 해결한 후 재시작하세요.")
            return

        amcl_launch = _find_amcl_launch()
        if amcl_launch is None:
            rospy.logwarn("amcl.launch 파일을 찾을 수 없습니다. AMCL을 시작하지 않습니다.")
            return

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

        self._pm.start("AMCL 스택", cmd)
        rospy.loginfo("AMCL 스택 시작 (map_file=%s)", map_file)

    # -- SLAM 공통 --

    def _start_slam_stack(self, name, find_launch_fn, extra_args=None):
        launch_file = find_launch_fn()
        if launch_file is None:
            rospy.logwarn("%s launch 파일을 찾을 수 없습니다.", name)
            return

        cmd = [
            "roslaunch", launch_file,
            f"robot_ip:={self.robot_ip}",
            f"robot_port:={self.robot_port}",
            "launch_rviz:=true",
            "launch_sensor_bridge:=false",
        ]
        if extra_args:
            cmd.extend(extra_args)

        self._pm.start(f"{name} 스택", cmd)

    def start_gmapping(self):
        self._start_slam_stack("GMapping", _find_gmapping_launch)

    def start_cartographer(self):
        self._start_slam_stack("Cartographer", _find_cartographer_launch)

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
            return

        launch_file = _find_cartographer_localization_launch()
        if launch_file is None:
            rospy.logwarn("cartographer_localization.launch 파일을 찾을 수 없습니다.")
            return

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
        self._pm.start(f"{stack_label} 스택", cmd)
        rospy.loginfo("Cartographer Localization 시작 [mode=%s] (state_file=%s)", mode, state_file)


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
    """`rviz_on` / `amcl` / `gmap` / `carto_map` / `carto_loc_fix` / `carto_loc_nonfix` 플래그를 분리하고 나머지는 ROS argv로 유지."""
    flags = {
        "rviz": False, "amcl": False, "gmap": False,
        "carto_map": False,
        "carto_loc_fix": False,    # 고정 맵 localization (AMCL 유사)
        "carto_loc_nonfix": False, # 서브맵 업데이트 포함 localization
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
    }

    for arg in argv[1:]:
        key = arg.lower()
        if key in FLAG_MAP:
            flags[FLAG_MAP[key]] = True
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

    Thread(target=_run_asyncio, daemon=True).start()
    rospy.Service('mobile_move', MoveMobile, service_handler)

    # 센서 브릿지는 항상 기동
    launcher.start_sensor_bridge()

    if flags["rviz"]:
        launcher.start_rviz(use_amcl_rviz=flags["amcl"])

    if flags["amcl"]:
        resolved_map = map_file or rospy.get_param(
            "~map_file", "/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_map.yaml"
        )
        launcher.start_amcl(resolved_map)

    if flags["gmap"]:
        launcher.start_gmapping()

    if flags["carto_map"]:
        launcher.start_cartographer()

    _carto_loc_mode = None
    if flags["carto_loc_fix"]:
        _carto_loc_mode = "fix"
    elif flags["carto_loc_nonfix"]:
        _carto_loc_mode = "nonfix"

    if _carto_loc_mode is not None:
        if state_file:
            # CLI에서 명시적으로 지정된 경우 그대로 사용
            resolved_state = state_file
        else:
            # _ensure_map_loaded()에서 선택된 맵 이름으로 .pbstream 자동 탐색
            rospy.loginfo("맵 선택 완료 대기 중... (최대 30초)")
            if controller is not None and controller.map_ready_event.wait(timeout=30.0):
                if controller.selected_map_name:
                    resolved_state = _find_pbstream_for_map(controller.selected_map_name)
                    if resolved_state:
                        rospy.loginfo("선택된 맵 '%s'에 대응하는 .pbstream 파일 발견: %s",
                                      controller.selected_map_name, resolved_state)
                    else:
                        rospy.logwarn("선택된 맵 '%s'에 대응하는 .pbstream 파일이 없습니다. 기본 경로 사용",
                                      controller.selected_map_name)
                        resolved_state = rospy.get_param(
                            "~state_file",
                            "/root/catkin_ws/src/TR-200/woosh_slam/maps/carto_woosh_map.pbstream"
                        )
                else:
                    rospy.logwarn("맵 선택이 완료되었으나 선택된 맵이 없습니다. 기본 경로 사용")
                    resolved_state = rospy.get_param(
                        "~state_file",
                        "/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_cartographer_map.pbstream"
                    )
            else:
                rospy.logwarn("맵 선택 대기 타임아웃. 기본 경로 사용")
                resolved_state = rospy.get_param(
                    "~state_file",
                    "/root/catkin_ws/src/TR-200/woosh_slam/maps/woosh_cartographer_map.pbstream"
                )
        launcher.start_cartographer_localization(resolved_state, mode=_carto_loc_mode)

    rospy.loginfo("서버 시작됨! (Quintic Minimum Jerk 정밀 제어)")
    rospy.loginfo("  rosservice call /mobile_move \"{distance: 0.3}\"")
    rospy.loginfo("  옵션: rviz_on | amcl [map_file:=...] | gmap | carto_map")
    rospy.loginfo("         carto_loc_fix [state_file:=...]      — 고정 맵 localization (AMCL 유사)")
    rospy.loginfo("         carto_loc_nonfix [state_file:=...]   — 서브맵 업데이트 + pose 보정")
    rospy.spin()


if __name__ == "__main__":
    main()
