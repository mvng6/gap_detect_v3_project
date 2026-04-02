# Changelog

> Generated: 2026-04-02
> Range: HEAD~30..HEAD (last 30 non-merge commits)

---

## Features

- `8ae2aac` Navigation 기능 구현
- `4251a02` navigation 로그 기능 추가
- `14aad27` costmap 기능 구현 완료 / costmap 기능 실행 시 rviz 출력 안되는 문제 해결 중
- `12797f4` nav_on 플래그 활용한 global costmap 기능 구현 중 (오류 수정 중)
- `70b8731` woosh_service_driver 코드 리팩토링 + 하드웨어·PC 저장 맵 목록 출력 및 선택 기능 구현
- `317bd8c` 라이다 센서 데이터 활용 패키지 분리
- `d54cd5e` cartographer, cartographer_ros 서브모듈 등록

## Bug Fixes

- `1d6c12b` 원인 2 문제 해결 수정 작업
- `8c74e16` SmoothTwistController + CmdVelAdapter 이중 WebSocket 연결 충돌 원인 관련 수정
- `f06baee` localization 시 사용되는 맵 선정 오류 수정 완료
- `1210021` 안전 마진 수정 중

## Refactor

- `5dec26a` Codex 활용 Navigation 구조 개선 완료
- `8f31ad2` Codex 활용 Navigation 관련 구조 수정 중
- `6d05a61` 불필요한 로그 제거
- `70b8731` woosh_service_driver 코드 리팩토링 (위 Features 항목과 중복)
- `df66f63` 리팩토링 프롬프트 추가
- `b76284c` Add English ROS1 navigation refactoring prompt

## Config / Params

- `26dfbe7` DWA 플래너 튜닝
- `7b7905e` DWA 파라미터 조정을 통한 구동 불안정 문제 해결 중

## Docs

- `816a4c5` PLAN 문서 작성
- `f5e8ebf` docs/config: Navigation 테스트 가이드 추가 및 파라미터 정리
- `659df50` 매뉴얼 추가 / global costmap AMCL 테스트 완료
- `5aa1dd6` woosh 로봇 SDK 매뉴얼 추가
- `55b4521` map 폴더 위치 변경 및 관련 코드 의존성 수정 / Readme 체크리스트 업데이트
- `c5a9026` 문서 업데이트

## Chore / Testing

- `22d522b` Phase 2 검증 진행 중
- `583ebfd` Phase 1 진행 완료
- `575914c` /odom 활용 수정
- `b9f1cc7` cartographer 맵 fix, nonfix 구동 테스트 완료
- `5c2d8b7` cartographer의 pbstream 형식 맵 저장 및 구동 테스트 완료
- `97f55af` cartographer 활용 맵 생성 테스트 완료
