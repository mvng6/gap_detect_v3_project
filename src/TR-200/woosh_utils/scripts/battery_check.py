#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모바일 로봇 배터리 잔량 확인 스크립트

ROS 노드로 실행 가능하며, ROS 파라미터를 통해 로봇 IP를 설정할 수 있습니다.

Usage:
    rosrun woosh_utils battery_check.py _robot_ip:=169.254.128.2 _robot_port:=5480
    
    또는 launch 파일에서:
    <node pkg="woosh_utils" type="battery_check.py" name="battery_check">
        <param name="robot_ip" value="169.254.128.2"/>
        <param name="robot_port" value="5480"/>
    </node>

Copyright © 2025 KATECH (Korea Automotive Technology Institute)
Author: LDJ (djlee2@katech.re.kr)
"""

import rospy
import asyncio
import sys
import os

# 소스 트리에서 직접 실행할 때도 SDK 모듈 경로를 찾을 수 있도록 보완
script_dir = os.path.dirname(os.path.abspath(__file__))
woosh_robot_dir = os.path.abspath(os.path.join(script_dir, "../../woosh_robot_py"))
if woosh_robot_dir not in sys.path:
    sys.path.insert(0, woosh_robot_dir)
woosh_utils_src_dir = os.path.abspath(os.path.join(script_dir, "../src"))
if woosh_utils_src_dir not in sys.path:
    sys.path.insert(0, woosh_utils_src_dir)

try:
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings
    from woosh.proto.robot.robot_pb2 import RobotInfo
except ImportError as e:
    rospy.logerr(f"Woosh SDK 모듈을 불러올 수 없습니다: {e}")
    rospy.logerr("`catkin build` 후 `source devel/setup.bash`가 적용됐는지 확인하세요.")
    rospy.logerr("직접 실행 중이라면 `src/TR-200/woosh_robot_py` 경로와 Python 의존성(websockets, protobuf 등)을 확인하세요.")
    sys.exit(1)

# woosh_utils 패키지가 아직 빌드/설치되지 않은 소스 트리에서도 import 가능하도록 보완
try:
    from woosh_utils import Colors, print_battery_status
except ImportError:
    from woosh_utils import Colors, print_battery_status
from woosh_utils import log_sdk_owner


async def check_battery_async(robot_ip, robot_port):
    """
    비동기 배터리 확인 함수
    
    Args:
        robot_ip: 로봇 IP 주소
        robot_port: 로봇 포트 번호
    """
    rospy.loginfo(f"{Colors.BOLD}🤖 모바일 로봇 연결 중... (IP: {robot_ip}, Port: {robot_port}){Colors.ENDC}")
    
    # 연결 설정
    settings = CommuSettings(
        addr=robot_ip,
        port=robot_port,
        identity="battery_checker"
    )
    robot = WooshRobot(settings)
    
    try:
        log_sdk_owner(
            rospy.loginfo,
            "open_start",
            "battery_check",
            "battery_checker",
            robot_ip,
            robot_port,
            "battery_check.py:check_battery_async",
        )
        await robot.run()
        log_sdk_owner(
            rospy.loginfo,
            "open_established",
            "battery_check",
            "battery_checker",
            robot_ip,
            robot_port,
            "battery_check.py:check_battery_async",
        )

        # 로봇 정보 요청 → 배터리 잔량 출력
        info, ok, msg = await robot.robot_info_req(RobotInfo())
        if not ok:
                rospy.logerr(f"{Colors.FAIL}{Colors.BOLD}❌ 로봇 정보 조회 실패: {msg}{Colors.ENDC}")
        else:
            # 배터리 정보를 색상으로 출력
            battery_level = info.battery.power
            print_battery_status(battery_level)

    except Exception as e:
        rospy.logerr(f"{Colors.FAIL}{Colors.BOLD}❌ 오류 발생: {e}{Colors.ENDC}")
    finally:
        # 연결 종료
        log_sdk_owner(
            rospy.loginfo,
            "close_start",
            "battery_check",
            "battery_checker",
            robot_ip,
            robot_port,
            "battery_check.py:check_battery_async",
        )
        await robot.stop()
        log_sdk_owner(
            rospy.loginfo,
            "close_complete",
            "battery_check",
            "battery_checker",
            robot_ip,
            robot_port,
            "battery_check.py:check_battery_async",
        )
        rospy.loginfo(f"{Colors.OKGREEN}✅ 연결 종료{Colors.ENDC}")


def main():
    """메인 함수 (ROS 노드)"""
    # ROS 노드 초기화
    rospy.init_node('battery_check', anonymous=False)
    
    # ROS 파라미터 로드
    robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
    robot_port = rospy.get_param('~robot_port', 5480)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("배터리 상태 확인 노드 시작")
    rospy.loginfo(f"로봇 IP: {robot_ip}")
    rospy.loginfo(f"로봇 포트: {robot_port}")
    rospy.loginfo("=" * 60)
    
    # 비동기 함수 실행
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(check_battery_async(robot_ip, robot_port))
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 중단되었습니다.")
    except Exception as e:
        rospy.logerr(f"예외 발생: {e}")
    finally:
        rospy.loginfo("배터리 확인 노드 종료")


if __name__ == "__main__":
    main()
