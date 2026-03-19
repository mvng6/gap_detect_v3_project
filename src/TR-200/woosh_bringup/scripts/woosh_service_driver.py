#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import asyncio
import math
import numpy as np
import sys
import os
import shutil
import signal
import subprocess
import atexit
from queue import Queue, Empty
from threading import Thread

# === Python 경로 설정 ===
# 현재 스크립트 디렉토리 기준으로 필요한 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))

# 소스 트리에서 직접 실행할 때도 SDK 모듈 경로를 찾을 수 있도록 보완
woosh_robot_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_robot_py"))
if woosh_robot_dir not in sys.path:
    sys.path.insert(0, woosh_robot_dir)

# woosh_utils 패키지가 아직 빌드/설치되지 않은 소스 트리에서도 import 가능하도록 보완
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
from woosh.proto.robot.robot_pack_pb2 import Twist, ExecTask
from woosh.proto.robot.robot_pb2 import (RobotInfo, PoseSpeed, OperationState, TaskProc, ScannerData)
from woosh.proto.robot.robot_pack_pb2 import SwitchMap, SetRobotPose, InitRobot, SwitchControlMode
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.task_pb2 import State as TaskState, Type as TaskType, Direction as TaskDirection

from woosh.proto.util.robot_pb2 import ControlMode

class SmoothTwistController:
    def __init__(self):
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_ctrl')

        self.robot = None

        # === 제어 파라미터 (작은 이동에 최적화) ===
        self.max_speed = 0.12      # 최대 속도 (절대값) - 작게 조정
        self.accel = 0.25          # 가속도
        self.decel = 0.50          # 감속도 - 충분히 강하게
        self.control_hz = 50       # 제어 주기 (50Hz) - 더 정밀한 제어

        # === 상태 (매번 초기화) ===
        self.target_distance = 0.0
        self.estimated_distance = 0.0
        self.current_speed = 0.0   # 부호 포함 (음수 = 후진)
        self.is_moving = False

        self.command_queue = Queue()
        self.result_queue = Queue()

    async def connect(self):
        settings = CommuSettings(addr=self.robot_ip, port=self.robot_port, identity=self.robot_identity)
        self.robot = WooshRobot(settings)
        await self.robot.run()

        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 연결 실패")
        
        # battery_check.py의 함수를 사용하여 배터리 상태 출력
        print_battery_status(info.battery.power)
        rospy.loginfo("로봇 연결 성공!")

        await self._setup_map()
    
    async def _setup_map(self):
        """네비게이션 설정: 맵 로드 및 로컬라이제이션"""
        rospy.loginfo("=== 네비게이션 설정 시작 ===")

        # map_loaded 변수를 함수 시작 부분에서 초기화
        map_loaded = False

        # 1단계: 현재 상태 확인
        rospy.loginfo("1단계: 현재 상태 확인")
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logwarn(f"위치 정보 요청 실패: {msg}")
            return False
        
        # 현재 맵 ID를 확인하여 맵 로드 여부 판단
        current_map_id = pose_speed.map_id if hasattr(pose_speed, 'map_id') else 0
        if current_map_id != 0:
            map_loaded = True
            rospy.loginfo(f"   현재 로드된 맵 ID: {current_map_id}")
        else:
            rospy.loginfo("   현재 로드된 맵이 없습니다.")

        # 2단계: 사용 가능한 맵 목록 확인
        rospy.loginfo("2단계: 사용 가능한 맵 목록 확인")
        scene_list_req = SceneList()
        scene_list, ok, msg = await self.robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        available_scenes = []
        if ok and scene_list and scene_list.scenes:
            for scene in scene_list.scenes:
                available_scenes.append(scene.name)
            rospy.loginfo(f"{len(available_scenes)}개의 장면을 찾았습니다:")
            for i, scene_name in enumerate(available_scenes, 1):
                rospy.loginfo(f"   {i}. {scene_name}")
        else:
            rospy.logwarn(f"맵 목록 확인 실패: {msg if not ok else '사용 가능한 맵이 없습니다.'}")

        # 3단계: 맵 로드 (맵이 로드되지 않은 경우)
        if not map_loaded and available_scenes:
            rospy.loginfo("3단계: 맵 로드")
            # 3번째 맵(인덱스 2)을 선택
            target_scene = available_scenes[2]
            rospy.loginfo(f"   맵 로드 시도: {target_scene}")
            
            switch_map = SwitchMap()
            switch_map.scene_name = target_scene
            result, ok, msg = await self.robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
            
            if ok:
                rospy.loginfo(f"맵 '{target_scene}' 로드 요청 성공")
                # await asyncio.sleep(3)  # 맵 로드 완료 대기
                
                # 맵 로드 확인
                pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
                if ok and pose_speed.map_id != 0:
                    rospy.loginfo(f"맵 ID가 {pose_speed.map_id}로 업데이트되었습니다.")
                    map_loaded = True
                else:
                    rospy.loginfo("ℹ맵 로드 요청 성공 (로컬라이제이션 대기 중)")
                    map_loaded = True  # 요청 성공했으므로 4단계 진행
            else:
                rospy.logerr(f"맵 로드 실패: {msg}")
        elif map_loaded:
            rospy.loginfo("맵이 이미 로드되어 있어 맵 로드를 건너뜁니다.")
        
        # 4단계: 로봇 위치 설정 (로컬라이제이션)
        if map_loaded:
            rospy.loginfo("4단계: 로봇 위치 설정 (로컬라이제이션)")
            pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok:
                # 현재 위치를 맵 상의 위치로 설정
                set_pose = SetRobotPose()
                set_pose.pose.x = pose_speed.pose.x
                set_pose.pose.y = pose_speed.pose.y
                set_pose.pose.theta = pose_speed.pose.theta
                
                result, ok, msg = await self.robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
                if ok:
                    rospy.loginfo(f"로봇 위치 설정 성공: ({set_pose.pose.x:.2f}, {set_pose.pose.y:.2f}, {set_pose.pose.theta:.2f})")
                    await asyncio.sleep(2)
                else:
                    rospy.logwarn(f"로봇 위치 설정 실패: {msg}")

        # 5단계: 로봇 초기화
        rospy.loginfo("5단계: 로봇 초기화")
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            init_robot = InitRobot()
            init_robot.is_record = False
            init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
            init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
            init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0
            
            result, ok, msg = await self.robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
            if ok:
                rospy.loginfo(f"✅ 로봇 초기화 성공: ({init_robot.pose.x:.2f}, {init_robot.pose.y:.2f}, {init_robot.pose.theta:.2f})")
                await asyncio.sleep(2)
            else:
                rospy.logwarn(f"⚠️ 로봇 초기화 실패: {msg}")
        
        # 6단계: 제어 모드를 자동 모드로 설정
        rospy.loginfo("6단계: 제어 모드를 자동 모드로 설정")
        switch_mode = SwitchControlMode()
        switch_mode.mode = ControlMode.kAuto
        result, ok, msg = await self.robot.switch_control_mode_req(switch_mode, NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo("✅ 자동 제어 모드 설정 성공")
            await asyncio.sleep(2)
        else:
            rospy.logwarn(f"⚠️ 제어 모드 설정 실패: {msg}")
        
        # 최종 상태 확인
        rospy.loginfo("최종 상태 확인")
        state, ok, msg = await self.robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if ok:
            # 디버그: state.robot과 state.nav 값 직접 출력
            rospy.loginfo(f"[DEBUG] state.robot = {state.robot} (이진: {bin(state.robot)})")
            rospy.loginfo(f"[DEBUG] state.nav = {state.nav} (이진: {bin(state.nav)})")
            rospy.loginfo(f"[DEBUG] kTaskable 값 = {OperationState.RobotBit.kTaskable}")
            rospy.loginfo(f"[DEBUG] state.robot & kTaskable = {state.robot & OperationState.RobotBit.kTaskable}")
            
            if state.robot & OperationState.RobotBit.kTaskable:
                rospy.loginfo("✅ 로봇이 작업을 받을 수 있는 상태입니다.")
            else:
                rospy.loginfo("ℹ️ 로봇이 아직 Taskable 상태가 아니지만, 작업 수행은 가능할 수 있습니다.")
            
            if state.nav & OperationState.NavBit.kImpede:
                rospy.logwarn("⚠️ 장애물이 감지되었습니다.")
            else:
                rospy.loginfo("✅ 네비게이션 경로가 깨끗합니다.")
        
        rospy.loginfo("=== 네비게이션 설정 완료 ===")
        return True

    def _quintic_minimum_jerk_profile(self, tau):
        """
        Minimum Jerk + Quintic Polynomial 속도 프로파일 계산
        
        5차 다항식 기반 Minimum Jerk Trajectory:
        - 위치: x(τ) = D * [10τ³ - 15τ⁴ + 6τ⁵]
        - 속도: v(τ) = (D/T) * [30τ² - 60τ³ + 30τ⁴]
        - 가속도: a(τ) = (D/T²) * [60τ - 180τ² + 120τ³]
        
        여기서 τ = t/T (정규화된 시간, 0~1)
        
        특징:
        - 시작/끝에서 속도 = 0, 가속도 = 0 보장
        - 저크(Jerk)를 최소화하여 덜컹거림 제거
        - 매우 부드러운 S자 형태의 속도 곡선
        
        Args:
            tau: 정규화된 시간 (0.0 ~ 1.0)
            
        Returns:
            tuple: (position_ratio, velocity_ratio, acceleration_ratio)
                   - position_ratio: 위치 비율 (0~1)
                   - velocity_ratio: 속도 비율 (최대 약 1.875)
                   - acceleration_ratio: 가속도 비율
        """
        # 범위 제한
        tau = np.clip(tau, 0.0, 1.0)
        
        tau2 = tau * tau       # τ²
        tau3 = tau2 * tau      # τ³
        tau4 = tau3 * tau      # τ⁴
        tau5 = tau4 * tau      # τ⁵
        
        # 위치 비율: 10τ³ - 15τ⁴ + 6τ⁵
        position_ratio = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
        
        # 속도 비율: 30τ² - 60τ³ + 30τ⁴ (위치의 미분)
        velocity_ratio = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4
        
        # 가속도 비율: 60τ - 180τ² + 120τ³ (속도의 미분)
        acceleration_ratio = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3
        
        return position_ratio, velocity_ratio, acceleration_ratio

    def _calculate_motion_time(self, distance):
        """
        이동 거리에 따른 최적 이동 시간 계산
        
        Quintic 프로파일에서 최대 속도는 τ=0.5에서 발생:
        v_max = (D/T) * [30*(0.5)² - 60*(0.5)³ + 30*(0.5)⁴]
              = (D/T) * [7.5 - 7.5 + 1.875]
              = (D/T) * 1.875
        
        따라서: T = 1.875 * D / v_max
        
        Args:
            distance: 이동 거리 (절대값)
            
        Returns:
            float: 계산된 총 이동 시간 (초)
        """
        abs_distance = abs(distance)
        
        # Quintic 프로파일의 최대 속도 계수 (τ=0.5에서)
        QUINTIC_PEAK_VELOCITY_RATIO = 1.875
        
        # 이론적 이동 시간 계산
        total_time = QUINTIC_PEAK_VELOCITY_RATIO * abs_distance / self.max_speed
        
        # 최소/최대 시간 제한
        min_time = 0.3  # 최소 0.3초 (너무 빠른 이동 방지)
        max_time = 30.0  # 최대 30초
        
        return np.clip(total_time, min_time, max_time)

    async def _move_exact_distance(self, distance):
        """
        Minimum Jerk + Quintic Polynomial 기반 부드러운 거리 이동
        
        5차 다항식을 사용하여 시작과 끝에서 속도, 가속도가 모두 0이 되어
        덜컹거림 없이 매우 부드럽게 이동합니다.
        
        수학적 배경:
        - 위치: x(τ) = D * [10τ³ - 15τ⁴ + 6τ⁵]
        - 속도: v(τ) = (D/T) * [30τ² - 60τ³ + 30τ⁴]
        - 가속도: a(τ) = (D/T²) * [60τ - 180τ² + 120τ³]
        
        Args:
            distance: 이동할 거리 (m), 양수=전진, 음수=후진
            
        Returns:
            tuple: (성공여부, 메시지)
        """
        # === 1. 너무 작은 거리는 무시 ===
        if abs(distance) < 0.005:
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            return True, f"완료: {distance:+.3f}m (너무 작음)"

        # === 2. 상태 초기화 ===
        self.target_distance = distance
        self.estimated_distance = 0.0
        self.current_speed = 0.0
        self.is_moving = True

        # === 3. 이동 파라미터 계산 ===
        period = 1.0 / self.control_hz
        direction = np.sign(distance)  # +1 (전진) or -1 (후진)
        abs_distance = abs(distance)
        
        # 최적 이동 시간 계산
        total_time = self._calculate_motion_time(abs_distance)
        
        rospy.loginfo(f"{'='*50}")
        rospy.loginfo(f"[Quintic Minimum Jerk] 이동 시작")
        rospy.loginfo(f"  - 목표 거리: {distance:+.3f}m ({'전진' if direction > 0 else '후진'})")
        rospy.loginfo(f"  - 예상 시간: {total_time:.2f}s")
        rospy.loginfo(f"  - 최대 속도: {self.max_speed:.3f}m/s")
        rospy.loginfo(f"{'='*50}")
        
        # === 4. 시간 기반 제어 루프 ===
        start_time = asyncio.get_event_loop().time()
        last_time = start_time
        last_log_time = start_time

        while self.is_moving:
            now = asyncio.get_event_loop().time()
            dt = max(now - last_time, period)
            last_time = now
            
            # 경과 시간 계산
            elapsed = now - start_time
            
            # 정규화된 시간 (0 ~ 1)
            tau = min(elapsed / total_time, 1.0)
            
            # === Quintic Minimum Jerk 프로파일 계산 ===
            pos_ratio, vel_ratio, acc_ratio = self._quintic_minimum_jerk_profile(tau)
            
            # 목표 속도 계산: v = (D/T) * velocity_ratio * direction
            target_speed = (abs_distance / total_time) * vel_ratio * direction
            
            # 속도 제한 (안전 장치)
            target_speed = np.clip(target_speed, -self.max_speed, self.max_speed)
            
            # === 부드러운 속도 전환 (2차 안전 필터) ===
            # 급격한 속도 변화 방지를 위한 추가 필터링
            max_speed_change = self.accel * dt * 2.0  # 여유 있는 가속도 제한
            speed_diff = target_speed - self.current_speed
            
            if abs(speed_diff) > max_speed_change:
                # 급격한 변화 시 제한된 속도로 전환
                self.current_speed += np.sign(speed_diff) * max_speed_change
            else:
                # 목표 속도로 직접 설정
                self.current_speed = target_speed
            
            # === 종료 조건 확인 ===
            if tau >= 1.0:
                self.current_speed = 0.0
                self.is_moving = False
            
            # === Twist 명령 전송 ===
            await self.robot.twist_req(
                Twist(linear=self.current_speed, angular=0.0), 
                NO_PRINT, NO_PRINT
            )
            
            # === 추정 거리 누적 ===
            self.estimated_distance += self.current_speed * dt
            
            # === 주기적 상태 로깅 (0.5초마다) ===
            if now - last_log_time >= 0.5:
                rospy.loginfo(
                    f"[진행] τ={tau:.2f}, 속도={self.current_speed:+.3f}m/s, "
                    f"추정거리={self.estimated_distance:+.3f}m"
                )
                last_log_time = now
            
            await asyncio.sleep(period)

        # === 5. 최종 정지 처리 ===
        # 안전을 위해 정지 명령을 여러 번 전송
        for _ in range(5):
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(period)
        
        self.is_moving = False
        
        # === 6. 결과 로깅 ===
        error = self.estimated_distance - distance
        rospy.loginfo(f"{'='*50}")
        rospy.loginfo(f"[Quintic Minimum Jerk] 이동 완료")
        rospy.loginfo(f"  - 목표 거리: {distance:+.3f}m")
        rospy.loginfo(f"  - 추정 거리: {self.estimated_distance:+.3f}m")
        rospy.loginfo(f"  - 오차: {error:+.4f}m ({abs(error)*1000:.1f}mm)")
        rospy.loginfo(f"{'='*50}")
        
        return True, f"완료: {self.estimated_distance:+.3f}m (오차: {error*1000:+.1f}mm)"

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


# 전역
controller = None
rviz_debug_process = None
rviz_process = None
amcl_process = None


def parse_cli_args(argv):
    """`rviz_on` / `amcl` 사용자 플래그를 분리하고 나머지는 ROS argv로 유지한다."""
    launch_rviz = False
    enable_amcl = False
    map_file = None
    filtered_argv = [argv[0]]

    for arg in argv[1:]:
        if arg.lower() in {"rviz_on", "--rviz_on"}:
            launch_rviz = True
            continue
        if arg.lower() in {"amcl", "--amcl"}:
            enable_amcl = True
            launch_rviz = True  # AMCL은 rviz_on을 내포
            continue
        if arg.lower().startswith("map_file:="):
            map_file = arg[len("map_file:="):]
            continue
        filtered_argv.append(arg)

    return launch_rviz, enable_amcl, map_file, filtered_argv


def find_package_dir():
    source_candidate = os.path.abspath(os.path.join(script_dir, ".."))
    if os.path.isfile(os.path.join(source_candidate, "package.xml")):
        return source_candidate

    try:
        import rospkg

        return rospkg.RosPack().get_path("woosh_bringup")
    except Exception:
        return source_candidate


def find_debug_script_path():
    candidates = [
        os.path.join(script_dir, "woosh_rviz_debug.py"),
        os.path.join(find_package_dir(), "scripts", "woosh_rviz_debug.py"),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    return None


def find_rviz_config_path():
    candidates = [
        os.path.abspath(os.path.join(script_dir, "../rviz/woosh_rviz_debug.rviz")),
        os.path.join(find_package_dir(), "rviz", "woosh_rviz_debug.rviz"),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    return None


def find_amcl_rviz_config_path():
    candidates = [
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/AMCL/rviz/amcl_debug.rviz")),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    try:
        import rospkg
        return os.path.join(
            rospkg.RosPack().get_path("woosh_slam_amcl"), "rviz", "amcl_debug.rviz"
        )
    except Exception:
        return None


def find_amcl_launch_path():
    candidates = [
        os.path.abspath(os.path.join(script_dir, "../../woosh_navigation/AMCL/launch/amcl.launch")),
    ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    try:
        import rospkg
        return os.path.join(
            rospkg.RosPack().get_path("woosh_slam_amcl"), "launch", "amcl.launch"
        )
    except Exception:
        return None


def find_rviz_binary():
    rviz_bin = shutil.which("rviz")
    if rviz_bin:
        return rviz_bin

    fallback = "/opt/ros/noetic/bin/rviz"
    if os.path.isfile(fallback):
        return fallback

    return None


def terminate_process_tree(process, name):
    if process is None:
        return

    if process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGTERM)
        rospy.loginfo("%s 종료 요청 완료", name)
    except ProcessLookupError:
        pass
    except Exception as exc:
        rospy.logwarn("%s 종료 중 예외: %s", name, exc)


def stop_amcl_support():
    global amcl_process
    terminate_process_tree(amcl_process, "AMCL 스택")
    amcl_process = None


def stop_rviz_support():
    global rviz_debug_process, rviz_process

    terminate_process_tree(rviz_process, "RViz")
    terminate_process_tree(rviz_debug_process, "RViz 디버그 노드")

    rviz_process = None
    rviz_debug_process = None

    stop_amcl_support()


def _validate_map_file(map_file):
    """맵 파일(.yaml)과 참조된 이미지(.pgm) 존재 여부를 검증한다.

    Returns:
        True: 정상, False: 파일 없음 (오류 로그 포함)
    """
    if not os.path.isfile(map_file):
        rospy.logerr("맵 파일을 찾을 수 없습니다: %s", map_file)
        rospy.logerr("  원인: Docker 볼륨 마운트가 적용되지 않았거나 컨테이너 밖에서 실행 중일 수 있습니다.")
        rospy.logerr("  해결:")
        rospy.logerr("    1. 컨테이너 재시작: docker-compose -f docker-compose.noetic_integration.yml up -d")
        rospy.logerr("    2. 컨테이너 진입:   docker exec -it noetic_robot_system_ws bash")
        rospy.logerr("    3. 맵 생성:         rosrun woosh_slam_amcl export_map.py _robot_ip:=169.254.128.2")
        rospy.logerr("    4. 다른 맵 지정:    rosrun woosh_bringup woosh_service_driver.py amcl map_file:=/path/to/map.yaml")
        return False

    # yaml 안의 image 경로도 확인
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


def start_amcl_support(robot_ip, robot_port, map_file):
    """AMCL 스택(sensor_bridge + map_server + amcl)을 roslaunch로 시작한다."""
    global amcl_process

    if not _validate_map_file(map_file):
        rospy.logerr("AMCL을 시작하지 않습니다. 위 오류를 해결한 후 재시작하세요.")
        return

    amcl_launch = find_amcl_launch_path()
    if amcl_launch is None:
        rospy.logwarn("amcl.launch 파일을 찾을 수 없습니다. AMCL을 시작하지 않습니다.")
        return

    cmd = [
        "roslaunch", amcl_launch,
        f"robot_ip:={robot_ip}",
        f"robot_port:={robot_port}",
        f"map_file:={map_file}",
        "launch_rviz:=false",   # RViz는 rviz_on 경로에서 별도 실행
    ]

    try:
        amcl_process = subprocess.Popen(cmd, start_new_session=True)
        rospy.loginfo("AMCL 스택 시작 (map_file=%s)", map_file)
    except Exception as exc:
        rospy.logwarn("AMCL 스택 시작 실패: %s", exc)


def start_rviz_support(use_amcl_rviz=False):
    global rviz_debug_process, rviz_process

    debug_script = find_debug_script_path()
    rviz_bin = find_rviz_binary()

    # AMCL 모드면 amcl_debug.rviz, 아니면 woosh_rviz_debug.rviz 사용
    if use_amcl_rviz:
        rviz_config = find_amcl_rviz_config_path()
        config_label = "amcl_debug.rviz"
    else:
        rviz_config = find_rviz_config_path()
        config_label = "woosh_rviz_debug.rviz"

    if debug_script is None:
        rospy.logwarn("`woosh_rviz_debug.py`를 찾지 못해 RViz 지원을 시작하지 않습니다.")
        return

    if rviz_config is None:
        rospy.logwarn("RViz 설정 파일(%s)을 찾지 못해 RViz 지원을 시작하지 않습니다.", config_label)
        return

    if rviz_bin is None:
        rospy.logwarn("`rviz` 실행 파일을 찾지 못했습니다. `rviz_on` 요청은 건너뜁니다.")
        return

    robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
    robot_port = rospy.get_param("~robot_port", 5480)

    debug_cmd = [
        sys.executable,
        debug_script,
        f"_robot_ip:={robot_ip}",
        f"_robot_port:={robot_port}",
        "_robot_identity:=rviz_debug",
    ]
    rviz_cmd = [rviz_bin, "-d", rviz_config]

    try:
        rviz_debug_process = subprocess.Popen(debug_cmd, start_new_session=True)
        rviz_process = subprocess.Popen(rviz_cmd, start_new_session=True)
        rospy.loginfo(
            "RViz 디버그 창 시작 (설정: %s)", config_label
        )
    except Exception as exc:
        rospy.logwarn("RViz 지원 시작 실패: %s", exc)
        stop_rviz_support()


def service_handler(req):
    global controller
    if controller is None:
        return MoveMobileResponse(False, "서버 초기화 중")

    controller.command_queue.put(req.distance)
    try:
        success, msg = controller.result_queue.get(timeout=15.0)
        return MoveMobileResponse(success, msg)
    except Empty:
        return MoveMobileResponse(False, "타임아웃")


def run_asyncio():
    global controller
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    controller = SmoothTwistController()

    async def main():
        try:
            await controller.run()
        except Exception as e:
            rospy.logerr(f"Asyncio 오류: {e}")

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()


if __name__ == "__main__":
    enable_rviz, enable_amcl, map_file, init_argv = parse_cli_args(sys.argv)

    rospy.init_node('mobile_move_server', anonymous=False, argv=init_argv)
    rospy.on_shutdown(stop_rviz_support)
    atexit.register(stop_rviz_support)

    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()

    rospy.Service('mobile_move', MoveMobile, service_handler)

    if enable_rviz:
        start_rviz_support(use_amcl_rviz=enable_amcl)

    if enable_amcl:
        robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
        robot_port = rospy.get_param("~robot_port", 5480)
        resolved_map = map_file or rospy.get_param(
            "~map_file", "/root/catkin_ws/src/TR-200/woosh_navigation/maps/woosh_map.yaml"
        )
        start_amcl_support(robot_ip, robot_port, resolved_map)

    rospy.loginfo("서버 시작됨! (정/역방향 정밀 제어 - 작은 이동 최적화)")
    rospy.loginfo("rosservice call /mobile_move \"{distance: 0.3}\"")
    rospy.loginfo("rosservice call /mobile_move \"{distance: -0.3}\"")
    rospy.loginfo("RViz를 함께 띄우려면: rosrun woosh_bringup woosh_service_driver.py rviz_on")
    rospy.loginfo("AMCL 로컬리제이션과 함께 시작하려면:")
    rospy.loginfo("  rosrun woosh_bringup woosh_service_driver.py amcl")
    rospy.loginfo("  rosrun woosh_bringup woosh_service_driver.py amcl map_file:=/path/to/map.yaml")
    rospy.spin()
