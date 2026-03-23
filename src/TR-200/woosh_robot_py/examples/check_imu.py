#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 장착 여부 및 작동 상태 확인 스크립트

HardwareState.imu 필드를 통해 로봇에 IMU가 장착되어 있는지,
현재 정상 작동 중인지 확인합니다.

실행 방법:
    # Docker 컨테이너 내부에서
    cd /root/catkin_ws/src/TR-200/woosh_robot_py
    python3 examples/check_imu.py --ip 169.254.128.2

    # 구독 모드로 5초간 변화 감시
    python3 examples/check_imu.py --ip 169.254.128.2 --subscribe --duration 5

주의: HardwareState.imu 는 IMU 측정값(가속도/각속도)이 아닌
      IMU 하드웨어 상태 코드(정상/경고/오류)만 제공합니다.

HardwareState.State 열거형 값:
    kNormal =  0  → 정상 (단, protobuf 기본값과 동일하여 미장착과 구분 불가)
    kInfo   =  1  → 정보 상태
    kWarn   = 16  → 경고
    kFatal  = 17  → 치명적 오류
"""

import asyncio
import argparse
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

from woosh_robot import WooshRobot  # noqa: E402
from woosh_interface import CommuSettings, NO_PRINT  # noqa: E402
from woosh.proto.robot.robot_pb2 import HardwareState, RobotInfo  # noqa: E402


# ── State 해석 헬퍼 ──────────────────────────────────────────────────────────

def _state_label(state: int) -> str:
    """State 정수값을 한국어 레이블로 변환."""
    return {
        HardwareState.kNormal: "정상 (kNormal)",
        HardwareState.kInfo:   "정보 (kInfo)",
        HardwareState.kWarn:   "경고 (kWarn)",
        HardwareState.kFatal:  "치명적 오류 (kFatal)",
    }.get(state, f"알 수 없음 (값={state})")


def _state_icon(state: int) -> str:
    return {
        HardwareState.kNormal: "[OK]",
        HardwareState.kInfo:   "[INFO]",
        HardwareState.kWarn:   "[WARN]",
        HardwareState.kFatal:  "[FAIL]",
    }.get(state, "[????]")


def print_imu_result(hw: HardwareState, source: str = "요청") -> None:
    """HardwareState 에서 IMU 상태를 출력."""
    imu_state = hw.imu
    icon = _state_icon(imu_state)
    label = _state_label(imu_state)

    print()
    print(f"=== IMU 상태 ({source}) ===")
    print(f"  {icon}  {label}")
    print()

    if imu_state == HardwareState.kNormal:
        print("  해석: kNormal(0)은 '정상' 또는 'protobuf 기본값(미보고)' 둘 다일 수 있습니다.")
        print("        - 로봇이 IMU 상태를 정상으로 보고 중 → IMU 장착 및 작동 정상")
        print("        - 또는 로봇 펌웨어가 IMU 상태를 아예 전송하지 않음 → 미장착 가능성")
        print("        구독 모드(--subscribe)로 추가 확인을 권장합니다.")
    elif imu_state == HardwareState.kInfo:
        print("  해석: IMU 장착 확인됨. 정보 수준 이벤트 발생 (정상 범위 내).")
    elif imu_state == HardwareState.kWarn:
        print("  해석: IMU 장착 확인됨. 경고 상태 — 보정 오류나 진동 과다 등을 점검하세요.")
    elif imu_state == HardwareState.kFatal:
        print("  해석: IMU 장착 확인됨. 치명적 오류 — 연결 불량 또는 하드웨어 고장 가능성.")
    print()

    # 참고: 다른 주요 하드웨어 상태도 함께 출력
    print("  --- 주요 하드웨어 상태 참고 ---")
    print(f"  board     : {_state_icon(hw.board)}  {_state_label(hw.board)}")
    print(f"  power     : {_state_icon(hw.power)}  {_state_label(hw.power)}")
    print(f"  magnetism : {_state_icon(hw.magnetism)}  {_state_label(hw.magnetism)}")
    if hw.motor:
        for i, m in enumerate(hw.motor):
            print(f"  motor[{i}]   : {_state_icon(m)}  {_state_label(m)}")
    print()


# ── 메인 로직 ────────────────────────────────────────────────────────────────

async def run(ip: str, port: int, use_subscribe: bool, duration: float) -> None:
    print(f"[연결] 로봇 {ip}:{port} 에 연결 중...")

    settings = CommuSettings(
        addr=ip,
        port=port,
        identity="imu_check",
    )
    robot = WooshRobot(settings)

    try:
        ok = await robot.run()
        if not ok:
            print("[오류] SDK 연결 실패.")
            return
        print("[연결] 연결 성공.")

        # ── 방법 1: 일회성 요청 ──────────────────────────────────────────
        print("\n[요청] HardwareState 단일 요청 중...")
        hw, ok, msg = await robot.robot_hardware_state_req(
            HardwareState(), NO_PRINT, NO_PRINT
        )
        if ok and hw:
            print_imu_result(hw, source="단일 요청")
        else:
            print(f"[오류] 단일 요청 실패: {msg}")

        # ── 방법 2 (선택): 구독으로 변화 감시 ───────────────────────────
        if use_subscribe:
            received_count = [0]

            def on_hardware_state(data: HardwareState) -> None:
                received_count[0] += 1
                print_imu_result(data, source=f"구독 #{received_count[0]}")

            print(f"[구독] HardwareState 구독 시작 — {duration}초간 감시...")
            await robot.robot_hardware_state_sub(on_hardware_state, NO_PRINT)
            await asyncio.sleep(duration)

            if received_count[0] == 0:
                print("[구독] 구독 기간 동안 업데이트 수신 없음.")
                print("       (로봇이 상태 변화 없이 안정적이거나, 구독 미지원일 수 있음)")
            else:
                print(f"[구독] 총 {received_count[0]}회 업데이트 수신 완료.")

    finally:
        try:
            await robot.stop()
        except Exception:
            pass
        print("[종료] 연결 종료.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Woosh TR-200 IMU 장착 여부 및 작동 상태 확인"
    )
    parser.add_argument("--ip",        default="169.254.128.2", help="로봇 IP 주소")
    parser.add_argument("--port",      default=5480, type=int,  help="로봇 포트")
    parser.add_argument("--subscribe", action="store_true",     help="구독 모드 활성화")
    parser.add_argument("--duration",  default=5.0, type=float, help="구독 감시 시간(초, 기본 5)")
    args = parser.parse_args()

    asyncio.run(run(args.ip, args.port, args.subscribe, args.duration))


if __name__ == "__main__":
    main()
