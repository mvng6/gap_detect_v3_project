#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Woosh TR-200 센서 브릿지 노드

Woosh SDK에서 레이저 스캔과 속도 데이터를 받아 ROS 표준 토픽으로 변환·발행합니다.
모든 모드(standalone, GMapping, Cartographer, AMCL)에서 /scan, /odom, TF(odom→base_link)를 제공합니다.

발행 토픽:
  /scan          (sensor_msgs/LaserScan)   — 레이저 스캔
  /odom_raw      (nav_msgs/Odometry)       — SDK PoseSpeed.pose 기반 원시 오도메트리 (IMU 퓨전 포함)
  TF             odom → base_link  (publish_tf:=true 일 때만)

TF 트리 (이 노드 기준):
  odom → base_link   (이 노드가 발행)
  base_link → laser  (launch 파일의 static_transform_publisher가 발행)
  map → odom         (amcl 노드가 발행)

Example:
  rosrun woosh_sensor_bridge woosh_sensor_bridge.py \\
    _robot_ip:=169.254.128.2 _robot_port:=5480
"""

import asyncio
import logging
import math
import os
import sys

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist as RosTwist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WOOSH_ROBOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../woosh_robot_py"))
if WOOSH_ROBOT_DIR not in sys.path:
    sys.path.insert(0, WOOSH_ROBOT_DIR)
WOOSH_UTILS_SRC_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "../../woosh_utils/src"))
if WOOSH_UTILS_SRC_DIR not in sys.path:
    sys.path.insert(0, WOOSH_UTILS_SRC_DIR)

from woosh_robot import WooshRobot  # noqa: E402
from woosh_interface import CommuSettings, NO_PRINT  # noqa: E402
from woosh_utils import log_sdk_owner  # noqa: E402
from woosh.proto.robot.robot_pb2 import PoseSpeed, RobotInfo, ScannerData  # noqa: E402


def _yaw_to_quaternion(yaw):
    """yaw 각도(rad)를 geometry_msgs/Quaternion으로 변환."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


class WooshSensorBridgeNode:
    def __init__(self):
        self.robot_ip = rospy.get_param("~robot_ip", "169.254.128.2")
        self.robot_port = rospy.get_param("~robot_port", 5480)
        self.robot_identity = rospy.get_param("~robot_identity", "amcl_bridge")

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.laser_frame = rospy.get_param("~laser_frame", "laser")

        self.publish_hz = max(1.0, float(rospy.get_param("~publish_hz", 10.0)))
        self.state_poll_sec = max(0.1, float(rospy.get_param("~state_poll_sec", 0.1)))
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.robot = None
        self.sdk_connected = False

        # 최신 SDK 데이터
        self.latest_scan = None
        self.latest_pose_speed = None

        # 오도메트리 상태 (odom 프레임 기준)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_twist_time = None

        # SDK PoseSpeed.pose 직접 활용을 위한 원점 기록
        # 로봇 내부 위치추정(IMU+wheel odom 퓨전)값을 odom 원점 기준 상대좌표로 변환
        self._sdk_pose_initialized = False
        self._pose_origin_x = 0.0
        self._pose_origin_y = 0.0
        self._pose_origin_theta = 0.0

        # 발행자
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom_raw", Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def _create_sdk_logger(self):
        logger = logging.getLogger(f"{self.robot_identity}.sdk")
        logger.setLevel(logging.CRITICAL)
        logger.propagate = False
        logger.handlers = []
        logger.addHandler(logging.NullHandler())
        return logger

    def _on_sdk_connection_change(self, connected):
        if self.sdk_connected == connected:
            return
        self.sdk_connected = connected
        if connected:
            rospy.loginfo("Woosh SDK 연결됨 (센서 브릿지).")
        else:
            rospy.logwarn("Woosh SDK 연결 끊김 (센서 브릿지).")

    def _on_scan(self, scan):
        self.latest_scan = scan

    def _on_pose(self, pose_speed):
        self._update_odom_from_sdk_pose(pose_speed)
        self.latest_pose_speed = pose_speed

    def _update_odom_from_sdk_pose(self, pose_speed):
        """SDK PoseSpeed.pose를 odom 상태에 반영한다.

        로봇 내부 위치추정값(wheel odom + IMU 퓨전 결과인 PoseSpeed.pose)을
        odom 프레임 원점 기준 상대좌표로 변환하여 사용한다.
        공식 ROS 인터페이스의 /odom (IMU fusion) 토픽과 동일한 데이터 소스이다.

        PoseSpeed.pose가 유효하지 않을 때(0,0,0)는 twist 적분 방식으로 폴백한다.
        """
        pose = pose_speed.pose

        # pose 유효성 검사: 로봇이 데이터를 아직 제공하지 않으면 (0,0,0)
        has_valid_pose = not (pose.x == 0.0 and pose.y == 0.0 and pose.theta == 0.0)

        if not has_valid_pose:
            # Fallback: twist 적분
            self._integrate_twist_fallback(pose_speed)
            return

        if not self._sdk_pose_initialized:
            # 첫 유효 pose를 odom 원점으로 설정
            self._pose_origin_x = pose.x
            self._pose_origin_y = pose.y
            self._pose_origin_theta = pose.theta
            self._sdk_pose_initialized = True
            rospy.loginfo(
                "SDK pose 원점 설정: x=%.3f y=%.3f theta=%.3f (rad)",
                pose.x, pose.y, pose.theta,
            )
            return

        # 원점 기준 상대 변위를 odom 프레임으로 변환
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

    def _integrate_twist_fallback(self, pose_speed):
        """PoseSpeed.pose가 없을 때 twist 적분으로 odom을 추정하는 폴백."""
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

        self.odom_theta = math.atan2(
            math.sin(self.odom_theta), math.cos(self.odom_theta)
        )

    async def connect(self):
        log_sdk_owner(
            rospy.loginfo,
            "open_start",
            "WooshSensorBridgeNode",
            self.robot_identity,
            self.robot_ip,
            self.robot_port,
            "woosh_sensor_bridge.py:WooshSensorBridgeNode.connect",
            note="standalone_sensor_bridge",
        )
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
            raise RuntimeError("Woosh SDK 연결 시작에 실패했습니다.")
        self.sdk_connected = True

        info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError(f"로봇 정보 조회 실패: {msg}")

        # woosh_service_driver가 선택한 맵이 있으면 우선 표시
        # (로컬 맵 선택 시 로봇 하드웨어 scene과 다를 수 있음)
        selected_map = rospy.get_param("/woosh/selected_map_name", "")
        selected_map_source = rospy.get_param("/woosh/selected_map_source", "")
        if selected_map and selected_map != info.scene.scene_name:
            rospy.loginfo(
                "센서 브릿지 연결 성공 (활성 맵: %s [%s] / robot scene: %s)",
                selected_map, selected_map_source, info.scene.scene_name,
            )
        else:
            rospy.loginfo(
                "센서 브릿지 연결 성공 (scene=%s, map=%s)",
                info.scene.scene_name,
                info.scene.map_name,
            )
        log_sdk_owner(
            rospy.loginfo,
            "open_established",
            "WooshSensorBridgeNode",
            self.robot_identity,
            self.robot_ip,
            self.robot_port,
            "woosh_sensor_bridge.py:WooshSensorBridgeNode.connect",
            note="standalone_sensor_bridge",
        )

        # 초기 포즈·스캔 데이터로 상태 초기화
        if info.pose_speed:
            self.latest_pose_speed = info.pose_speed
            self.last_twist_time = rospy.get_time()
            # 초기 SDK pose를 odom 원점으로 미리 설정
            init_pose = info.pose_speed.pose
            if init_pose.x != 0.0 or init_pose.y != 0.0 or init_pose.theta != 0.0:
                self._pose_origin_x = init_pose.x
                self._pose_origin_y = init_pose.y
                self._pose_origin_theta = init_pose.theta
                self._sdk_pose_initialized = True
                rospy.loginfo(
                    "초기 SDK pose 원점 설정: x=%.3f y=%.3f theta=%.3f (rad)",
                    init_pose.x, init_pose.y, init_pose.theta,
                )

        # 구독 등록
        await self.robot.robot_pose_speed_sub(self._on_pose, NO_PRINT)
        await self.robot.scanner_data_sub(self._on_scan, NO_PRINT)

        # 초기 폴링
        await self._request_pose_once()
        await self._request_scan_once()

    async def _request_pose_once(self):
        try:
            pose_speed, ok, _ = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            if ok and pose_speed:
                self._update_odom_from_sdk_pose(pose_speed)
                self.latest_pose_speed = pose_speed
        except Exception:
            pass

    async def _request_scan_once(self):
        try:
            scan, ok, _ = await self.robot.scanner_data_req(
                ScannerData(), NO_PRINT, NO_PRINT
            )
            if ok and scan:
                self.latest_scan = scan
        except Exception:
            pass

    def _publish_scan(self):
        if self.latest_scan is None:
            return

        s = self.latest_scan
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.laser_frame

        msg.angle_min = s.angle_min
        msg.angle_max = s.angle_max
        msg.angle_increment = s.angle_increment
        msg.time_increment = s.time_increment
        msg.scan_time = s.scan_time
        msg.range_min = s.range_min
        msg.range_max = s.range_max
        msg.ranges = list(s.ranges)
        # intensities 미제공

        self.scan_pub.publish(msg)

    def _publish_odom_and_tf(self):
        now = rospy.Time.now()
        quat = _yaw_to_quaternion(self.odom_theta)

        # TF: odom → base_link (publish_tf 파라미터로 제어 — EKF 사용 시 false로 설정)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        # 위치 공분산: x, y, yaw 불확실도
        odom.pose.covariance[0] = 0.05   # x
        odom.pose.covariance[7] = 0.05   # y
        odom.pose.covariance[35] = 0.1   # yaw

        if self.latest_pose_speed is not None:
            odom.twist.twist.linear.x = self.latest_pose_speed.twist.linear
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.latest_pose_speed.twist.angular
        # 속도 공분산
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.005

        self.odom_pub.publish(odom)

    async def spin(self):
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

    async def shutdown(self):
        if self.robot is not None:
            log_sdk_owner(
                rospy.loginfo,
                "close_start",
                "WooshSensorBridgeNode",
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                "woosh_sensor_bridge.py:WooshSensorBridgeNode.shutdown",
            )
            try:
                await self.robot.stop()
            except Exception as exc:
                rospy.logwarn("SDK 종료 중 예외: %s", exc)
            log_sdk_owner(
                rospy.loginfo,
                "close_complete",
                "WooshSensorBridgeNode",
                self.robot_identity,
                self.robot_ip,
                self.robot_port,
                "woosh_sensor_bridge.py:WooshSensorBridgeNode.shutdown",
            )


def main():
    rospy.init_node("woosh_sensor_bridge", anonymous=False)
    node = WooshSensorBridgeNode()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(node.connect())
        loop.run_until_complete(node.spin())
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료되었습니다.")
    except Exception as exc:
        rospy.logerr("센서 브릿지 노드 예외: %s", exc)
    finally:
        loop.run_until_complete(node.shutdown())
        loop.close()


if __name__ == "__main__":
    main()
