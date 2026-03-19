#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from socket import timeout
import rospy
from std_msgs.msg import Float64

from dsr_msgs.srv import MoveJoint, MoveJointRequest
from woosh_msgs.srv import MoveMobile, MoveMobileRequest

def mobile_move(distance=0.1):
    """
    모바일 로봇 이동 함수
    
    Args:
        distance: 이동 거리 [m]
    Returns:
        bool: 성공 여부
    """
    service_name = '/mobile_move'
    rospy.loginfo("Waiting for mobile service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_mobile = rospy.ServiceProxy(service_name, MoveMobile)
        req = MoveMobileRequest()
        req.distance = distance

        rospy.loginfo("Sending mobile move command: %.2f m", distance)
        response = move_mobile(req)
        rospy.loginfo("Mobile move command sent successfully.")
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Mobile service call failed: %s", e)
        return False
    
def dsr_move_joint(pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   vel=30.0, acc=30.0, time=0.0,
                   radius=0.0, mode=0, blendType=0, syncType=0):
    """
    DSR 로봇 조인트 이동 함수
    
    Args:
        pos: 목표 조인트 각도 [deg]
        vel: 속도 [%]
        acc: 가속도 [%]
        time: 이동 시간 [s]
        radius: 블렌딩 반경
        mode: 이동 모드 (0: 절대 위치)
        blendType: 블렌딩 타입
        syncType: 동기화 타입
    Returns:
        bool: 성공 여부
    """
    service_name = '/dsr01a0912/motion/move_joint'
    rospy.loginfo("Waiting for DSR service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_joint = rospy.ServiceProxy(service_name, MoveJoint)
        req = MoveJointRequest()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = radius
        req.mode = mode
        req.blendType = blendType
        req.syncType = syncType

        rospy.loginfo("Sending DSR move_joint command: %s", pos)
        resp = move_joint(req)
        if resp.success:
            rospy.loginfo("DSR move_joint succeeded.")
        else:
            rospy.logwarn("DSR move_joint failed: %s", resp.message if hasattr(resp, 'message') else "Unknown")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("DSR service call failed: %s", e)
        return False

def execute_dsr_motion(target_joint, dsr_vel, dsr_acc, dsr_time, description=""):
    """
    DSR 로봇 이동을 실행하고 결과를 반환하는 헬퍼 함수
    
    Args:
        target_joint: 목표 조인트 각도
        dsr_vel: 속도
        dsr_acc: 가속도
        dsr_time: 시간
        description: 동작 설명
    Returns:
        bool: 성공 여부
    """
    if description:
        rospy.loginfo("=== %s ===", description)
    
    if not dsr_move_joint(
        pos=target_joint,
        vel=dsr_vel,
        acc=dsr_acc,
        time=dsr_time,
        radius=0.0,
        mode=0,        # 0: 절대 위치
        blendType=0,
        syncType=0
    ):
        rospy.logerr("DSR motion failed.")
        return False
    
    rospy.sleep(1.0)  # 안정적인 전이 대기
    return True

def main():
    rospy.init_node('integrated_robot_client', anonymous=True)

    # 갭단차 측정 반복 횟수
    activate_iteration = 1

    # DSR 로봇 속도, 가속도, 이동 시간 설정
    dsr_vel = 30.0
    dsr_acc = 30.0
    dsr_time = 3.0

    # 홈 포지션 정의
    HOME_POSITION = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]
    
    # 측정 포인트 정의
    A_POINTS = [
        [49.54, 31.27, 87.67, 0.0, 61.06, -130.46],   # A-point1
        [55.63, 42.22, 68.88, 0.0, 68.9, -124.37],    # A-point2
        [60.42, 56.73, 41.99, 0.0, 81.28, -119.58],   # A-point3
    ]
    
    B_POINTS = [
        [62.64, 55.81, 37.25, 0.0, 86.94, -117.36],   # B-point1
        [69.94, 48.67, 50.55, 0.0, 80.78, -110.06],   # B-point2
        [80.45, 44.08, 58.82, 0.0, 77.11, -99.55],    # B-point3
        [90.3, 43.41, 60.0, 0.0, 76.59, -89.7],       # B-point4
    ]
    
    C_POINTS = [
        [72.49, 47.05, 53.49, 0.0, 79.46, -107.51],   # C-point1
        [76.66, 45.15, 56.92, 0.0, 77.94, -103.34],   # C-point2
        [88.41, 43.26, 60.27, 0.0, 76.47, -91.59],    # C-point3
        [99.88, 45.7, 55.92, 0.0, 78.38, -80.12],     # C-point4
        [107.71, 50.37, 47.44, 0.0, 82.19, -72.29],   # C-point5
    ]

    for i in range(activate_iteration):
        # 1. 두산로봇 홈 포지션 이동
        if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
                                "=== Step 1: Moving to home position ==="):
                return
        rospy.sleep(1.0)    # 안정화 대기

        # 2. 두산로봇 A-points 경로 구동
        rospy.loginfo("=== Step 2: Moving to A-points ===")
        for idx, point in enumerate(A_POINTS, 1):
            if not execute_dsr_motion(point, dsr_vel, dsr_acc, dsr_time, 
                                    f"2-{idx}. DSR 로봇 A-point{idx} 측정 위치"):
                return
        
        # 3. 두산로봇 홈 포지션 복귀
        rospy.loginfo("=== Step 3: Returning to home position ===")
        if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
                                "3. DSR 로봇 홈 포지션 복귀"):
            return

        # 4. 모바일 로봇 이동 (첫 번째)
        rospy.loginfo("=== Step 3: Moving mobile robot forward ===")
        mobile_distance = 0.3  # [m]
        if not mobile_move(mobile_distance):
            rospy.logerr("Aborting: Mobile robot failed to move.")
            return
        rospy.loginfo("Mobile robot movement completed.")
        rospy.sleep(1.0)  # 안정적인 전이 대기
        rospy.loginfo("\n>>> Gap Detection Sequence %d 완료!", i + 1)

    rospy.loginfo("=" * 60)
    rospy.loginfo("Entire sequence completed successfully!")
    rospy.loginfo("=" * 60)
    rospy.loginfo("All operations completed successfully.")

    # for i in range(activate_iteration):
    #     # 1. 두산로봇 홈 포지션 이동
    #     if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
    #                                 "=== Step 1: Moving to home position ==="):
    #         return
    
    #     rospy.sleep(1.0)    # 안정화 대기

    #     # 2. A-points 측정 위치들
    #     rospy.loginfo("=== Step 2: Moving to A-points ===")
    #     for idx, point in enumerate(A_POINTS, 1):
    #         if not execute_dsr_motion(point, dsr_vel, dsr_acc, dsr_time, 
    #                                    f"2-{idx}. DSR 로봇 A-point{idx} 측정 위치"):
    #             return

    #     # 3. 두산로봇 홈 포지션으로 복귀
    #     rospy.loginfo("=== Step 3: Returning to home position ===")
    #     if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
    #                                "3. DSR 로봇 홈 포지션 복귀"):
    #         return

    #     # 4. 모바일 로봇 이동 (첫 번째)
    #     rospy.loginfo("=== Step 4: Moving mobile robot forward ===")
    #     mobile_distance = 0.3  # [m]
    #     if not mobile_move(mobile_distance):
    #         rospy.logerr("Aborting: Mobile robot failed to move.")
    #         return
    #     rospy.loginfo("Mobile robot movement completed.")
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     # 5. B-points 측정 위치들
    #     rospy.loginfo("=== Step 5: Moving to B-points ===")
    #     for idx, point in enumerate(B_POINTS, 1):
    #         if not execute_dsr_motion(point, dsr_vel, dsr_acc, dsr_time, 
    #                                    f"5-{idx}. DSR 로봇 B-point{idx} 측정 위치"):
    #             return

    #     # 6. 두산로봇 홈 포지션으로 복귀
    #     rospy.loginfo("=== Step 6: Returning to home position ===")
    #     if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
    #                                "6. DSR 로봇 홈 포지션 복귀"):
    #         return
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     # 7. 모바일 로봇 이동 (두 번째)
    #     rospy.loginfo("=== Step 7: Moving mobile robot forward ===")
    #     mobile_distance = 0.6  # [m]
    #     if not mobile_move(mobile_distance):
    #         rospy.logerr("Aborting: Mobile robot failed to move.")
    #         return
    #     rospy.loginfo("Mobile robot movement completed.")
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     # 8. C-points 측정 위치들
    #     rospy.loginfo("=== Step 8: Moving to C-points ===")
    #     for idx, point in enumerate(C_POINTS, 1):
    #         if not execute_dsr_motion(point, dsr_vel, dsr_acc, dsr_time, 
    #                                    f"8-{idx}. DSR 로봇 C-point{idx} 측정 위치"):
    #             return
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     # 9. 두산로봇 홈 포지션으로 복귀
    #     rospy.loginfo("=== Step 9: Returning to home position ===")
    #     if not execute_dsr_motion(HOME_POSITION, dsr_vel, dsr_acc, dsr_time, 
    #                                "9. DSR 로봇 홈 포지션 복귀"):
    #         return
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     # 10. 모바일 로봇 이동 (초기 위치)
    #     rospy.loginfo("=== Step 10: Moving mobile robot backward ===")
    #     mobile_distance = -0.9  # [m]
    #     if not mobile_move(mobile_distance):
    #         rospy.logerr("Aborting: Mobile robot failed to move.")
    #         return
    #     rospy.loginfo("Mobile robot movement completed.")
    #     rospy.sleep(1.0)  # 안정적인 전이 대기

    #     rospy.loginfo("\n>>> Gap Detection Sequence %d 완료!", i + 1)

    # rospy.loginfo("=" * 60)
    # rospy.loginfo("Entire sequence completed successfully!")
    # rospy.loginfo("=" * 60)
    # rospy.loginfo("All operations completed successfully.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user.")
