#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
배터리 관련 유틸리티 함수

배터리 상태 표시 및 터미널 색상 관련 함수를 제공합니다.
"""


class Colors:
    """터미널 ANSI 색상 코드"""

    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def print_battery_status(battery_level):
    """
    배터리 잔량을 색상으로 표시

    Args:
        battery_level: 배터리 잔량 (%)
    """
    print("\n" + "=" * 60)

    if battery_level >= 80:
        color = Colors.OKGREEN
        icon = "🔋"
        status = "충분"
    elif battery_level >= 50:
        color = Colors.OKCYAN
        icon = "🔋"
        status = "보통"
    elif battery_level >= 20:
        color = Colors.WARNING
        icon = "🪫"
        status = "주의"
    else:
        color = Colors.FAIL
        icon = "🪫"
        status = "위험"

    bar_length = 40
    filled_length = int(bar_length * battery_level / 100)
    bar = "█" * filled_length + "░" * (bar_length - filled_length)

    print(f"{Colors.BOLD}{Colors.HEADER}📊 모바일 로봇 배터리 상태{Colors.ENDC}")
    print("=" * 60)
    print(f"\n{color}{Colors.BOLD}{icon}  배터리 잔량: {battery_level}% ({status}){Colors.ENDC}")
    print(f"\n[{color}{bar}{Colors.ENDC}] {battery_level}%\n")
    print("=" * 60 + "\n")
