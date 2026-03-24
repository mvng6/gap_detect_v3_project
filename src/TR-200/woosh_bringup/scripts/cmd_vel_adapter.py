#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cmd_vel_adapter.py — /cmd_vel → Woosh SDK twist_req() 어댑터 노드

ROS navigation stack(move_base 등)이 발행하는 /cmd_vel 을 수신하여
Woosh TR-200 SDK의 twist_req()로 전달합니다.

기능:
  - /cmd_vel (geometry_msgs/Twist) 구독
  - 속도 클리핑: max_linear 0.12 m/s, max_angular 0.5 rad/s
  - Watchdog: /cmd_vel 수신 중단 1.0초 후 자동 정지 명령 전송
  - asyncio 루프에서 SDK 호출, ROS 콜백과 Queue로 통신

사용 예:
  rosrun woosh_bringup cmd_vel_adapter.py _robot_ip:=169.254.128.2
  roslaunch woosh_bringup cmd_vel_adapter.launch robot_ip:=169.254.128.2
"""

import asyncio
import logging
import os
import sys
import time
from queue import Empty, Queue
from threading import Lock

import rospy
from geometry_msgs.msg import Twist as RosTwist

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WOOSH_ROBOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../woosh_robot_py"))
if WOOSH_ROBOT_DIR not in sys.path:
    sys.path.insert(0, WOOSH_ROBOT_DIR)

from woosh_robot import WooshRobot  # noqa: E402
from woosh_interface import CommuSettings, NO_PRINT  # noqa: E402
from woosh.proto.robot.robot_pack_pb2 import Twist  # noqa: E402
from woosh.proto.robot.robot_pb2 import RobotInfo  # noqa: E402


class CmdVelAdapter:
    def __init__(self):
        self.robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
        self.robot_port = rospy.get_param("~robot_port", 5480)
        self.robot_identity = rospy.get_param("~robot_identity", "cmd_vel_adapter")

        self.max_linear = rospy.get_param("~max_linear", 0.12)    # m/s
        self.max_angular = rospy.get_param("~max_angular", 0.5)   # rad/s
        self.watchdog_timeout = rospy.get_param("~watchdog_timeout", 1.0)  # seconds
        self.control_hz = rospy.get_param("~control_hz", 20.0)

        self.robot = None
        self._sdk_connected = False

        # ROS 콜백 → asyncio 루프 간 통신용 큐
        # 항목: (linear, angular) 튜플 또는 None (정지)
        self._cmd_queue = Queue(maxsize=1)
        self._last_cmd_lock = Lock()
        self._last_cmd_time = None   # 마지막 /cmd_vel 수신 시각 (time.monotonic)
        self._last_linear = 0.0
        self._last_angular = 0.0

        self._cmd_sub = rospy.Subscriber(
            "/cmd_vel", RosTwist, self._cmd_vel_callback, queue_size=1
        )

    def _create_sdk_logger(self):
        logger = logging.getLogger(f"{self.robot_identity}.sdk")
        logger.setLevel(logging.CRITICAL)
        logger.propagate = False
        logger.handlers = []
        logger.addHandler(logging.NullHandler())
        return logger

    def _on_sdk_connection_change(self, connected):
        if self._sdk_connected == connected:
            return
        self._sdk_connected = connected
        if connected:
            rospy.loginfo("[cmd_vel_adapter] SDK 연결됨.")
        else:
            rospy.logwarn("[cmd_vel_adapter] SDK 연결 끊김.")

    def _cmd_vel_callback(self, msg: RosTwist):
        """ROS 콜백 — /cmd_vel 수신 시 최신 명령을 큐에 넣는다."""
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        # 속도 클리핑
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        with self._last_cmd_lock:
            self._last_cmd_time = time.monotonic()
            self._last_linear = linear
            self._last_angular = angular

        # 큐가 꽉 차 있으면 기존 값 버리고 최신 값으로 교체
        try:
            self._cmd_queue.get_nowait()
        except Empty:
            pass
        try:
            self._cmd_queue.put_nowait((linear, angular))
        except Exception:
            pass

    async def _connect(self):
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity,
            logger=self._create_sdk_logger(),
            log_level="CRITICAL",
            log_to_console=False,
            log_to_file=False,
            connect_status_callback=self._on_sdk_connection_change,
        )
        self.robot = WooshRobot(settings)

        ok = await self.robot.run()
        if not ok:
            raise RuntimeError("Woosh SDK 연결 시작 실패.")
        self._sdk_connected = True

        _, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError(f"로봇 정보 조회 실패: {msg}")

        rospy.loginfo("[cmd_vel_adapter] 로봇 연결 성공. /cmd_vel 수신 대기 중.")

    async def _send_twist(self, linear=0.0, angular=0.0):
        if self.robot is None:
            return
        try:
            await self.robot.twist_req(
                Twist(linear=linear, angular=angular), NO_PRINT, NO_PRINT
            )
        except Exception as exc:
            rospy.logwarn("[cmd_vel_adapter] twist_req 실패: %s", exc)

    async def _spin(self):
        period = 1.0 / self.control_hz
        watchdog_fired = False

        while not rospy.is_shutdown():
            # 큐에서 최신 명령 꺼내기 (non-blocking)
            linear, angular = 0.0, 0.0
            got_cmd = False
            try:
                linear, angular = self._cmd_queue.get_nowait()
                got_cmd = True
            except Empty:
                pass

            # Watchdog: 마지막 수신 이후 timeout 경과 시 정지
            with self._last_cmd_lock:
                last_time = self._last_cmd_time
                if not got_cmd:
                    linear = self._last_linear
                    angular = self._last_angular

            if last_time is not None:
                elapsed = time.monotonic() - last_time
                if elapsed >= self.watchdog_timeout:
                    if not watchdog_fired:
                        rospy.logwarn(
                            "[cmd_vel_adapter] /cmd_vel %.1f초 미수신 — 자동 정지.",
                            self.watchdog_timeout,
                        )
                        watchdog_fired = True
                    linear, angular = 0.0, 0.0
                else:
                    watchdog_fired = False

            await self._send_twist(linear, angular)
            await asyncio.sleep(period)

        # 노드 종료 시 정지 명령 전송
        rospy.loginfo("[cmd_vel_adapter] 종료 — 정지 명령 전송.")
        for _ in range(3):
            await self._send_twist(0.0, 0.0)
            await asyncio.sleep(0.05)

    async def _shutdown(self):
        if self.robot is not None:
            try:
                await self.robot.stop()
            except Exception as exc:
                rospy.logwarn("[cmd_vel_adapter] SDK 종료 중 예외: %s", exc)


def main():
    rospy.init_node("cmd_vel_adapter", anonymous=False)
    node = CmdVelAdapter()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(node._connect())
        loop.run_until_complete(node._spin())
    except KeyboardInterrupt:
        rospy.loginfo("[cmd_vel_adapter] 사용자에 의해 종료.")
    except Exception as exc:
        rospy.logerr("[cmd_vel_adapter] 예외 발생: %s", exc)
    finally:
        loop.run_until_complete(node._shutdown())
        loop.close()


if __name__ == "__main__":
    main()
