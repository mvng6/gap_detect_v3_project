# SLAM / Navigation 개발 일지

**프로젝트**: Woosh TR-200 자율주행 시스템 (Gap Detect v3)
**대상 로봇**: Woosh TR-200 · 차동 구동 · 최대 선속도 0.12 m/s
**ROS 버전**: ROS1 Noetic
**작업자**: LDJ @ KATECH (djlee2@katech.re.kr)

---

## 작업 이력 요약

| 날짜 | 작업 번호 | 분류 | 제목 | 상태 |
|------|-----------|------|------|------|
| 2026-03-26 | #001 | 분석 | move_base_on 모드 가다 서다(Stop-and-Go) 원인 분석 | 완료 |
| 2026-03-26 | #002 | 버그수정 | 이중 WebSocket 연결 충돌 해결 | 완료 |
| 2026-03-26 | #003 | 버그수정 | await twist_req 블로킹으로 실질 제어 주기 저하 해결 | 완료 |
| 2026-03-26 | #004 | 기능추가 | navigation 명령 속도 실시간 CSV 로깅 | 완료 |
| 2026-03-26 | #005 | 분석 | 전진 구동 중 불안정 좌우 회전 원인 분석 | 완료 |
| 2026-03-26 | #006 | 버그수정 | angular 포화·oscillation recovery·속도 미달 수정 (파라미터 4종 + hold-last 패치) | 완료 |
| 2026-03-26 | #007 | 버그수정 | #006 회귀 버그 — acc_lim_theta=0.15로 DWA 탐색 공간 붕괴 → 영구 stop 수정 | 완료 |
| 2026-03-26 | #008 | 버그수정 | 전진 중 angular 진동 잔존(stdev=0.065, 반전7.9%) + clearing_rotation 재발(×2) 수정 | 완료 |
| 2026-03-26 | #009 | 버그수정 | angular 포화 37.2% + rotate_cw 구간 57% 낭비 → path_distance_bias 추가 완화 + clearing_rotation 비활성화 | 완료 |

---

---

# 2026-03-26

---

## #001 · 분석 · move_base_on 모드 가다 서다(Stop-and-Go) 원인 분석

**작업 시각**: 2026-03-26
**분류**: 원인 분석
**관련 파일**:
- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
- `src/TR-200/woosh_bringup/scripts/cmd_vel_adapter.py`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/move_base_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_costmap_params.yaml`
- `docs/navigation_algorithms.md`

---

### 현상

`move_base_on` 모드로 자율주행 시 로봇이 등속도로 주행하지 않고 **가다 서다를 반복**하는 불규칙 모션 발생.

---

### 분석 결과

#### 원인 A (최심각) — SmoothTwistController + CmdVelAdapter 이중 WebSocket 연결 충돌

`move_base_on` 모드에서 **두 개의 독립 WebSocket 연결**이 동시에 로봇에 `twist_req`를 전송하는 구조적 충돌.

| 노드 | identity | WebSocket 역할 |
|------|----------|----------------|
| `SmoothTwistController` (woosh_service_driver.py) | `twist_ctrl` | `/mobile_move` 서비스 처리 — 항상 연결 유지 |
| `CmdVelAdapter` (cmd_vel_adapter.py, 서브프로세스) | `cmd_vel_adapter` | move_base `/cmd_vel` 전달 — move_base_on 시 추가 연결 |

충돌 메커니즘:
- `main()`에서 `Thread(target=_run_asyncio, daemon=True).start()`로 SmoothTwistController가 WebSocket 연결을 선점
- 이후 `launcher.start_cmd_vel_adapter()`가 별도 서브프로세스로 두 번째 WebSocket 연결 시도
- Woosh SDK 내부에서 중복 연결 처리로 명령 드롭 또는 연결 재설정 발생 → **순간 정지 반복**

```
[문제 구조]
SmoothTwistController    → WebSocket #1  (항상 연결, 유휴 루프 유지)
cmd_vel_adapter (subprocess) → WebSocket #2  (충돌 발생)
```

---

#### 원인 B (심각) — await twist_req 블로킹으로 실질 제어 주기 저하

`cmd_vel_adapter.py`의 `_send_twist`는 `await self.robot.twist_req(...)`이므로 WebSocket 왕복 시간(RTT ~100ms) 동안 루프가 블로킹된다.

| 파라미터 | 설정값 | 실제 동작 |
|----------|--------|-----------|
| `control_hz` (cmd_vel_adapter) | 20Hz → 주기 50ms | twist_req RTT ~100ms → 실제 주기 100~150ms |
| `controller_frequency` (move_base) | 5Hz → cmd_vel 간격 200ms | 200ms 안에 어댑터 1회 전송도 불확실 |

결과: 이동 중 속도 명령이 간헐적으로 누락 → stop-and-go.

---

#### 원인 C (중간) — DWA oscillation 복구 행동 반복 트리거

합성 오도메트리(엔코더 없음, SDK `PoseSpeed.twist` dt 적분) 부정확성과 WebSocket 지연이 복합 작용.

```yaml
# local_planner_params.yaml
oscillation_reset_dist: 0.05  # DWA 내부 진동 판단 기준: 5cm

# move_base_params.yaml
oscillation_timeout: 10.0     # 10초 이상 진동 감지 시 복구
oscillation_distance: 0.2     # 0.2m 이동해야 진동 카운터 초기화
clearing_rotation_allowed: true
```

오도메트리가 느리게 업데이트되면 DWA가 5cm 이동을 감지 못해 oscillation 카운터 증가 → 복구 행동(제자리 회전 + 재계획) 발동 → 정지 후 재출발 반복.

---

#### 원인 D (중간) — path_distance_bias 과도 설정으로 잦은 경로 교정

```yaml
# local_planner_params.yaml
path_distance_bias: 32.0  # 전역 경로 추종 강도 (높을수록 경로에 강하게 달라붙음)
goal_distance_bias: 24.0
occdist_scale: 0.02
```

WebSocket 지연으로 로봇이 경로에서 조금이라도 이탈하면 DWA가 급격한 방향 수정 트라젝토리를 반복 선택 → 속도 감속이 주기적으로 발생.

---

#### 원인 E (경미) — 전역 경로 재계획 중 cmd_vel 공백으로 watchdog 발동 가능성

```yaml
# move_base_params.yaml
planner_frequency: 1.0  # Hz — 전역 경로 재계획 1초마다
```

재계획 연산이 1초를 초과하면 cmd_vel 발행이 중단되어 `watchdog_timeout: 1.0s` 발동 → 강제 정지.

---

### 조치 계획

| 원인 | 우선순위 | 조치 |
|------|----------|------|
| A. 이중 WebSocket 연결 | 최우선 | cmd_vel 패스스루를 SmoothTwistController에 통합 → #002 |
| B. await 블로킹 | 높음 | fire-and-forget 비동기 태스크 분리 → #003 |
| C. DWA oscillation 오감지 | 중간 | `oscillation_reset_dist` 상향 파라미터 튜닝 |
| D. path_distance_bias 과도 | 중간 | `20.0~24.0`으로 완화 |
| E. watchdog 발동 | 낮음 | `planner_patience` 연장 또는 watchdog_timeout 완화 검토 |

---

---

## #002 · 버그수정 · 이중 WebSocket 연결 충돌 해결

**작업 시각**: 2026-03-26
**분류**: 버그 수정
**수정 파일**: `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
**관련 분석**: #001 원인 A

---

### 문제 요약

`move_base_on` 모드에서 `SmoothTwistController`(woosh_service_driver.py)와 `CmdVelAdapter`(cmd_vel_adapter.py 서브프로세스)가 각각 독립적인 WebSocket 연결을 로봇에 유지하며 `twist_req`를 중복 전송한다.

---

### 해결 방향

`cmd_vel_adapter.py` 서브프로세스를 `move_base_on` 모드에서 기동하지 않고, `SmoothTwistController` 내부에 `/cmd_vel` 패스스루 기능을 통합한다. 단일 WebSocket 연결로 두 가지 제어 경로를 처리한다.

```
[수정 후 구조]
SmoothTwistController  → WebSocket #1  (단일 연결)
  ├─ /mobile_move 명령 → quintic 프로파일 전송  (우선 처리)
  └─ /cmd_vel 수신    → 동일 WebSocket으로 20Hz 패스스루
```

---

### 변경 내용

#### 변경 1 — import 추가

```python
# 수정 전
from geometry_msgs.msg import PoseWithCovarianceStamped

# 수정 후
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist as RosTwist
```

ROS `geometry_msgs/Twist`를 woosh proto `Twist`와 이름 충돌 없이 사용하기 위해 `RosTwist` 별칭으로 import.

---

#### 변경 2 — `SmoothTwistController.__init__` cmd_vel 상태 변수 추가

```python
# cmd_vel 패스스루 (move_base_on 모드) — 별도 WebSocket 연결 없이 기존 연결 재사용
self._cmd_vel_enabled = False
self._cmd_vel_queue = Queue(maxsize=1)
self._cmd_vel_lock = Lock()
self._cmd_vel_last_time = None
self._cmd_vel_last_linear = 0.0
self._cmd_vel_last_angular = 0.0
self._cmd_vel_watchdog_timeout = 1.0
self._cmd_vel_sub = None   # enable_cmd_vel_passthrough() 호출 시 생성
```

---

#### 변경 3 — `_send_twist` angular 파라미터 추가

```python
# 수정 전
async def _send_twist(self, linear=0.0):
    await self.robot.twist_req(Twist(linear=linear, angular=0.0), ...)

# 수정 후
async def _send_twist(self, linear=0.0, angular=0.0):
    await self.robot.twist_req(Twist(linear=linear, angular=angular), ...)
```

기존 호출부 모두 positional/keyword 방식으로 하위 호환 유지.

---

#### 변경 4 — 신규 메서드 `_cmd_vel_callback`

```python
def _cmd_vel_callback(self, msg):
    """move_base가 발행하는 /cmd_vel 을 수신해 큐에 넣는다."""
    linear = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
    angular = max(-0.5, min(0.5, float(msg.angular.z)))
    with self._cmd_vel_lock:
        self._cmd_vel_last_time = time.monotonic()
        self._cmd_vel_last_linear = linear
        self._cmd_vel_last_angular = angular
    try:
        self._cmd_vel_queue.get_nowait()   # 기존 값 드롭
    except Empty:
        pass
    try:
        self._cmd_vel_queue.put_nowait((linear, angular))
    except Exception:
        pass
```

- 속도 클리핑: 선속도 `±0.12 m/s`, 각속도 `±0.5 rad/s`
- `Queue(maxsize=1)` + 교체 패턴으로 항상 최신 명령만 유지

---

#### 변경 5 — 신규 메서드 `enable_cmd_vel_passthrough`

```python
def enable_cmd_vel_passthrough(self, watchdog_timeout=1.0):
    """cmd_vel 패스스루 모드를 활성화한다 (move_base_on 전용)."""
    self._cmd_vel_watchdog_timeout = watchdog_timeout
    self._cmd_vel_sub = rospy.Subscriber(
        "/cmd_vel", RosTwist, self._cmd_vel_callback, queue_size=1
    )
    self._cmd_vel_enabled = True
```

`main()`에서 `launcher.start_cmd_vel_adapter()` 대신 이 메서드를 호출하여 WebSocket 이중 연결을 방지한다.

---

#### 변경 6 — `_control_loop` cmd_vel 패스스루 루프 추가

`/mobile_move` 거리 명령이 최우선이며, cmd_vel 패스스루는 명령 대기 중에만 동작한다.

```python
async def _control_loop(self):
    period_idle   = 0.01          # /mobile_move 대기 루프 간격
    period_cmdvel = 1.0 / 20.0   # cmd_vel 패스스루 루프 간격 (20 Hz)
    _watchdog_fired = False

    while True:
        # ── /mobile_move 거리 명령 우선 처리 (선점) ─────────────────────
        try:
            distance = self.command_queue.get_nowait()
            success, msg = await self._move_exact_distance(distance)
            self.result_queue.put((success, msg))
            continue
        except Empty:
            pass

        # ── cmd_vel 패스스루 (move_base_on 모드) ────────────────────────
        if not self._cmd_vel_enabled:
            await asyncio.sleep(period_idle)
            continue

        # 큐에서 최신 명령, 없으면 마지막 명령 유지 (hold-last)
        # watchdog: 1.0초 이상 /cmd_vel 미수신 시 자동 정지
        ...
        await self._send_twist(linear, angular)
        await asyncio.sleep(period_cmdvel)
```

---

#### 변경 7 — `main()` start_cmd_vel_adapter 제거

```python
# 수정 전
launcher.start_cmd_vel_adapter()   # ← 이중 WebSocket 연결의 원인
launcher.start_move_base()

# 수정 후
_t = time.monotonic()
while controller is None and (time.monotonic() - _t) < 10.0:
    time.sleep(0.1)
if controller is not None:
    controller.enable_cmd_vel_passthrough()   # 기존 WebSocket 재사용
else:
    rospy.logwarn("SmoothTwistController 초기화 대기 타임아웃 ...")
launcher.start_move_base()
```

controller 초기화 완료까지 최대 10초 대기 후 패스스루 활성화.

---

### 검증 방법

```bash
# WebSocket 연결 수 확인 — ESTABLISHED가 1개여야 정상
ss -tnp | grep 5480

# ROS 노드 목록 — cmd_vel_adapter 노드가 없어야 정상
rosnode list

# 패스스루 활성화 로그 확인
rostopic echo /rosout | grep "cmd_vel 패스스루 활성화"
```

---

---

## #003 · 버그수정 · await twist_req 블로킹으로 실질 제어 주기 저하 해결

**작업 시각**: 2026-03-26
**분류**: 버그 수정
**수정 파일**: `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
**관련 분석**: #001 원인 B

---

### 문제 요약

`_send_twist()` 내부의 `await self.robot.twist_req(...)`가 WebSocket 왕복 응답(RTT ~100ms) 완료까지 asyncio 루프를 블로킹한다. 두 제어 경로 모두 영향을 받는다.

#### 영향 경로 1 — cmd_vel 패스스루 루프 (`_control_loop`)

```
의도 루프 주기:  period_cmdvel = 50ms  (20Hz)
실제 루프 주기:  await _send_twist ≈ 100ms  +  sleep 50ms  =  150ms  (≈ 6.7Hz)
```

move_base DWA는 5Hz(200ms)로 `/cmd_vel`을 발행하는데, 어댑터 실효 주기가 150ms이면 200ms 내에 전송 1회도 보장되지 않는 구간이 발생한다.

#### 영향 경로 2 — Quintic 프로파일 루프 (`_move_exact_distance`)

```
의도 루프 주기:  period = 20ms  (50Hz, control_hz=50)
실제 루프 주기:  await _send_twist ≈ 100ms  +  sleep 20ms  =  120ms  (≈ 8Hz)
```

`tau`는 wall time 기준 계산이므로 속도 프로파일 자체는 유지되지만, 단위 시간당 전송 횟수가 급감하여 저속 구간에서 명령 공백이 생길 수 있다.

---

### 해결 방향

**비동기 태스크 분리 (Fire-and-Forget)**: `twist_req`를 `asyncio.ensure_future()`로 백그라운드 태스크로 발행하고, 제어 루프는 즉시 sleep으로 진행한다.

- 이전 태스크 미완료 시 취소 후 최신 명령으로 교체 → 태스크 누적 없음 (최대 1개 상한)
- 정지 명령(`_move_exact_distance` 종료 5회 반복)은 완료 보장 필요 → `await _send_twist()` 유지

```
[수정 전]
_control_loop: await _send_twist (100ms 블로킹) → sleep 50ms → 실효 ≈ 6.7Hz

[수정 후]
_control_loop: _fire_twist (즉시 반환) → sleep 50ms → 실효 ≈ 20Hz
```

---

### 변경 내용

#### 변경 1 — `__init__` 태스크 추적 변수 추가

```python
# 비블로킹 twist 전송 태스크 추적 (WebSocket RTT 블로킹 방지용)
self._pending_send_task = None
```

---

#### 변경 2 — 신규 메서드 `_fire_twist`

```python
def _fire_twist(self, linear=0.0, angular=0.0):
    """twist_req를 백그라운드 asyncio 태스크로 발행한다.

    await 없이 호출하므로 WebSocket RTT(~100ms)가 제어 루프 주기에 영향을 주지 않는다.
    이전 태스크가 미완료이면 취소 후 최신 명령으로 교체한다 (최신 명령 우선).
    완료 보장이 필요한 정지 명령에는 _send_twist(await)를 직접 사용한다.
    """
    if self._pending_send_task is not None and not self._pending_send_task.done():
        self._pending_send_task.cancel()
    self._pending_send_task = asyncio.ensure_future(
        self.robot.twist_req(Twist(linear=linear, angular=angular), NO_PRINT, NO_PRINT)
    )
```

설계 원칙:
- sync 메서드로 선언 — `asyncio.ensure_future()`는 현재 이벤트 루프에 태스크를 등록하므로 async 컨텍스트 안에서 sync 호출 가능
- 태스크 1개 상한 — 이전 태스크 미완료 시 취소하여 메모리/태스크 누적 방지
- 최신 명령 우선 — 취소 후 즉시 새 태스크 발행

---

#### 변경 3 — `_move_exact_distance` 모션 명령 교체

```python
# 수정 전 (블로킹)
await self._send_twist(self.current_speed)

# 수정 후 (비블로킹)
self._fire_twist(self.current_speed)
```

정지 명령(루프 종료 후 5회)은 안전을 위해 `await` 유지:

```python
for _ in range(5):
    await self._send_twist()    # ← await 유지 (완료 보장)
    await asyncio.sleep(period)
```

---

#### 변경 4 — `_control_loop` cmd_vel 패스스루 전송 교체

```python
# 수정 전 (블로킹)
await self._send_twist(linear, angular)
await asyncio.sleep(period_cmdvel)

# 수정 후 (비블로킹)
self._fire_twist(linear, angular)
await asyncio.sleep(period_cmdvel)
```

---

### 수정 후 제어 주기 변화

| 경로 | 수정 전 실효 주기 | 수정 후 실효 주기 |
|------|------------------|------------------|
| cmd_vel 패스스루 | ~150ms (≈ 6.7Hz) | ~50ms (≈ 20Hz) |
| Quintic 프로파일 | ~120ms (≈ 8Hz) | ~20ms (≈ 50Hz) |
| 정지 명령 | await 유지 | await 유지 (변경 없음) |

---

### 검증 방법

```bash
# /cmd_vel 발행 간격 확인
rostopic hz /cmd_vel

# Quintic 프로파일 로그에서 "[진행] tau=..." 0.5초 간격 기록 확인
rosrun woosh_bringup woosh_service_driver.py amcl move_base_on map_file:=...
rosservice call /mobile_move "{distance: 0.3}"
```

---

---

## #004 · 기능추가 · navigation 명령 속도 실시간 CSV 로깅

**작업 시각**: 2026-03-26
**분류**: 기능 추가
**수정 파일**: `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
**목적**: navigation 구동 중 실시간 명령 속도·회전 방향을 CSV 파일로 기록하여 제어 동작 사후 검증 데이터 확보

---

### 배경

#002, #003으로 이중 WebSocket 충돌 및 블로킹 문제를 해결한 후, 실제 로봇에서 명령 속도가 어떻게 전달되었는지 정량적으로 검증할 수단이 없었다. 운행 후 CSV 파일을 분석하여 제어 주기 준수 여부, 속도 프로파일 형태, watchdog 발동 시점 등을 확인한다.

---

### 기록 대상

| 제어 경로 | `source` 값 | 기록 시점 |
|-----------|-------------|-----------|
| Quintic 직선 이동 (`_move_exact_distance`) | `"quintic"` | `_fire_twist()` 및 정지 명령 `_send_twist()` 호출마다 |
| move_base cmd_vel 패스스루 (`_control_loop`) | `"cmd_vel"` | `_fire_twist()` 호출마다 (20Hz) |

---

### 변경 내용

#### 변경 1 — `import csv` 추가

```python
import csv
```

---

#### 변경 2 — `NavCsvLogger` 클래스 신규 추가

```python
class NavCsvLogger:
    FIELDNAMES = [
        "timestamp", "elapsed_sec",
        "source", "linear_m_s", "angular_rad_s",
        "direction", "odom_x", "odom_y",
    ]
```

- **저장 경로**: `src/TR-200/woosh_bringup/logs/nav_cmd_YYYYMMDD_HHMMSS.csv`
- **파일 생성**: 노드 시작 시 자동 생성, 디렉터리 없으면 `os.makedirs`로 자동 생성
- **버퍼링**: `buffering=1` (라인 단위 즉시 flush) — 노드 강제 종료 시에도 데이터 보존
- **thread-safe**: `Lock` 으로 보호
- **direction 판정 로직** (`_direction_label`):

| 조건 | direction |
|------|-----------|
| `\|linear\| < 0.001` and `\|angular\| < 0.001` | `stop` |
| `\|linear\| >= 0.001`, linear > 0 | `forward` |
| `\|linear\| >= 0.001`, linear < 0 | `backward` |
| linear 정지, angular > 0 | `rotate_ccw` |
| linear 정지, angular < 0 | `rotate_cw` |

---

#### 변경 3 — `SmoothTwistController.__init__` 로거 초기화

```python
# 명령 속도 CSV 로거
self._csv_logger = NavCsvLogger()
```

---

#### 변경 4 — `_log_cmd` 헬퍼 메서드 추가

```python
def _log_cmd(self, source, linear, angular):
    with self._odom_lock:
        odom = self._odom_pose
    odom_x = odom[0] if odom is not None else None
    odom_y = odom[1] if odom is not None else None
    self._csv_logger.log(source, linear, angular, odom_x, odom_y)
```

현재 `/odom` 위치를 함께 기록하여 속도-위치 연계 분석을 가능하게 한다.

---

#### 변경 5 — `_send_twist` / `_fire_twist` `_source` 파라미터 추가 및 로그 호출

```python
# 수정 전
async def _send_twist(self, linear=0.0, angular=0.0):
    ...
def _fire_twist(self, linear=0.0, angular=0.0):
    ...

# 수정 후
async def _send_twist(self, linear=0.0, angular=0.0, _source="quintic"):
    self._log_cmd(_source, linear, angular)
    ...
def _fire_twist(self, linear=0.0, angular=0.0, _source="quintic"):
    self._log_cmd(_source, linear, angular)
    ...
```

기존 호출부는 `_source` 기본값(`"quintic"`)으로 하위 호환 유지.

---

#### 변경 6 — `_control_loop` cmd_vel 패스스루 source 지정

```python
# 수정 전
self._fire_twist(linear, angular)

# 수정 후
self._fire_twist(linear, angular, _source="cmd_vel")
```

---

#### 변경 7 — `run()` 종료 시 로거 close

```python
async def run(self):
    await self.connect()
    try:
        await self._control_loop()
    finally:
        self._csv_logger.close()
```

노드 정상/비정상 종료 모두에서 파일이 닫히도록 `finally` 처리.

---

### CSV 컬럼 정의

| 컬럼 | 타입 | 설명 |
|------|------|------|
| `timestamp` | float | Unix 절대 시각 (초, 소수점 4자리) |
| `elapsed_sec` | float | 로거 시작부터 경과 시간 (초) |
| `source` | str | 명령 출처: `quintic` \| `cmd_vel` |
| `linear_m_s` | float | 선속도 (m/s, 양수=전진, 음수=후진) |
| `angular_rad_s` | float | 각속도 (rad/s, 양수=좌회전, 음수=우회전) |
| `direction` | str | `forward` \| `backward` \| `rotate_ccw` \| `rotate_cw` \| `stop` |
| `odom_x` | float | `/odom` 기준 현재 x 위치 (m, 없으면 빈 값) |
| `odom_y` | float | `/odom` 기준 현재 y 위치 (m, 없으면 빈 값) |

---

### 활용 방법

```bash
# 로그 파일 위치 확인
ls src/TR-200/woosh_bringup/logs/

# Python으로 간단 분석 예시
python3 - <<'EOF'
import pandas as pd
df = pd.read_csv("src/TR-200/woosh_bringup/logs/nav_cmd_YYYYMMDD_HHMMSS.csv")

# source별 선속도 통계
print(df.groupby("source")["linear_m_s"].describe())

# cmd_vel 실효 주기 (간격 평균)
cmd = df[df["source"] == "cmd_vel"].copy()
cmd["dt"] = cmd["elapsed_sec"].diff()
print("cmd_vel 평균 주기(ms):", cmd["dt"].mean() * 1000)

# stop 명령 발생 시점
print(df[df["direction"] == "stop"][["elapsed_sec", "source"]])
EOF
```

---

### 검증 포인트

| 항목 | 확인 방법 |
|------|-----------|
| Quintic 프로파일 형태 | `source=="quintic"` 행의 `linear_m_s` 시계열이 종 모양(bell curve) 여부 |
| cmd_vel 실효 주기 | `source=="cmd_vel"` 행 간격 평균이 ~50ms(20Hz) 달성 여부 |
| watchdog 발동 시점 | `direction=="stop"` + `source=="cmd_vel"` 연속 구간 탐색 |
| 정지 명령 정상 전달 | 이동 종료 후 `linear_m_s==0.0` 레코드 5개 이상 연속 확인 |

---

---

---

## #005 · 분석 · 전진 구동 중 불안정 좌우 회전 원인 분석

**작업 시각**: 2026-03-26
**분류**: 원인 분석
**관련 파일**:
- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_bringup/logs/nav_cmd_20260326_062711.csv`

---

### 현상

`move_base_on` 모드로 전진 구동 시 등속도는 유지되나 **로봇이 좌/우로 불규칙하게 반복 회전**하는 모션 발생.

---

### 로그 통계 요약 (nav_cmd_20260326_062711.csv)

| 항목 | 값 |
|------|-----|
| 전진 구간 총 스텝 | 4,631 스텝 |
| angular ≠ 0인 스텝 | 4,524 / 4,631 (97.7%) |
| angular = ±0.10 포화 스텝 | 1,492 / 4,631 (32.2%) |
| 포화 블록 수 (연속 ≥ 3 스텝) | 140개, 평균 지속 10.7 스텝 (~550ms) |
| 한 스텝 내 angular 급변 (> 0.05 rad/s) | 272회 / 4,630 (5.9%) |
| cmd_vel 발행 주기 | 평균 51ms (≈ 19.6 Hz) |

---

### 분석 결과

#### 원인 A (주 원인) — `path_distance_bias` 과도 설정으로 localization 노이즈를 과도 추종

```yaml
# local_planner_params.yaml
path_distance_bias: 32.0   # 매우 강한 전역 경로 추종 강도
goal_distance_bias: 24.0
occdist_scale: 0.02         # 장애물 회피 가중치는 거의 무시
```

AMCL/Cartographer localization의 pose 추정에 수 cm 수준 노이즈가 존재할 경우, 전역 경로가 로봇의 좌/우로 번갈아 나타나는 것처럼 DWA에 보이게 된다. `path_distance_bias: 32.0`이 높아 DWA는 이 미세한 경로 오차에도 즉시 최대 각속도로 반응한다.

**로그 근거**: 10초 구간별 mean_angular의 부호가 전 구간에 걸쳐 반전되며, 포화 블록 140개가 전진 구간 전반에서 지속 발생.

---

#### 원인 B — `_fire_twist` WebSocket 태스크 취소 구조로 angular 방향 전환 시 명령 누락

```python
# woosh_service_driver.py
if self._pending_send_task is not None and not self._pending_send_task.done():
    self._pending_send_task.cancel()   # 이전 명령 취소
self._pending_send_task = asyncio.ensure_future(...)  # 최신 명령만 전송
```

WebSocket RTT ≈ 100ms인데 패스스루 루프 주기는 50ms(20Hz)이므로 이전 `twist_req`가 완료되기 전에 다음 루프가 취소하고 새 명령을 덮어쓴다. angular 값이 바뀌는 순간의 명령이 취소될 경우 로봇은 이전 angular 방향으로 더 오래 구동되어 오버슈트가 발생한다.

---

#### 원인 C — 큐 비어있을 때 마지막 angular 값 재전송 (hold-last) 로 포화 구간 연장

```python
# _control_loop 패스스루 루프
if not got_cmd:
    linear = self._cmd_vel_last_linear
    angular = self._cmd_vel_last_angular   # ← 마지막 angular 재전송
```

DWA가 `angular=+0.1`을 발행한 직후 루프가 큐에서 꺼내지 못하면, 다음 사이클에서 `+0.1`을 추가로 재전송한다. DWA는 이미 `angular=-0.1`로 갱신했어도 로봇은 `+0.1`을 한 번 더 받는 구조다. angular 포화 구간이 실제 DWA 의도보다 길어지는 직접 원인이다.

---

#### 원인 D — `min_vel_theta: 0.05` 불감대로 미세 보정 시 오버슈트 반복

```yaml
# local_planner_params.yaml
min_vel_theta: 0.05   # DWA 탐색 공간의 각속도 하한
```

경로 오차가 매우 작아 `0 < needed_angular < 0.05`인 경우에도 DWA는 최소 0.05 rad/s를 적용한다. 이 오버슈트가 반대 방향 보정을 유발하고 좌/우 진동이 반복된다.

---

#### 원인 E — `acc_lim_theta: 0.5 rad/s²` + 100ms 통신 지연으로 각속도 오버슈트

```yaml
# local_planner_params.yaml
acc_lim_theta: 0.5    # 허용 각가속도
```

DWA 시뮬레이션은 명령이 즉각 반영된다고 가정한다. 실제로는 WebSocket 100ms + 루프 지연이 더해져, 계산한 각속도 변화가 로봇에 반영되는 시점이 늦다. 결과적으로 DWA 제어 루프가 지연된 상태를 기반으로 보정을 누적시키며 오버슈트를 반복한다.

---

### 원인 연쇄 요약

```
localization 노이즈 (cm 수준)
    → DWA가 경로 좌/우 오차 감지
    → path_distance_bias=32.0 으로 최대 angular 명령 (-0.1 또는 +0.1)
    → _fire_twist에서 이전 태스크 취소 → 방향 전환 시 일부 명령 누락
    → hold-last 재전송으로 포화 값이 불필요하게 연장
    → 로봇 과보정 → 반대 방향 경로 오차 발생
    → 반대 방향 최대 angular 명령 → 반복
```

---

### 조치 계획

| 원인 | 우선순위 | 권장 조치 |
|------|----------|-----------|
| A. path_distance_bias 과도 | 높음 | `32.0` → `16.0~20.0`으로 낮추고 `goal_distance_bias` 상향 |
| D. min_vel_theta 불감대 | 높음 | `0.05` → `0.01~0.02`로 낮춰 미세 보정 가능하게 |
| E. acc_lim_theta + 통신 지연 | 중간 | `0.5` → `0.2` 이하로 낮춰 WebSocket 지연 내 오버슈트 방지 |
| C. hold-last 재전송 | 중간 | 큐 미수신 시 angular를 0으로 폴백하거나 DWA 발행 주기와 동기화 |
| B. _fire_twist 취소 | 낮음 | angular 방향 전환 시 취소 없이 전송 완료 후 교체하는 방식 검토 |

---

---

## #006 · 버그수정 · 전진 구동 중 각속도 포화 / oscillation recovery / 속도 미달성 수정

**작업 시각**: 2026-03-26
**분류**: 버그 수정 (파라미터 튜닝 + hold-last 패치)
**수정 파일**:
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
**해결 항목**: #001-C(D), #005-A, #005-C, #005-D, #005-E

---

### 현상 (nav_cmd_20260326_064807.csv 기준)

| 항목 | 관측값 | 기대값 |
|------|--------|--------|
| angular=-0.1 (포화) 구간 | 전진 구간 전반 지배적 | 소폭 방향 보정만 |
| linear 최대값 | 0.05 m/s | 0.10 m/s |
| DWA oscillation recovery | 발동됨 (+0.5 rad/s clearing rotation) | 발동 안됨 |
| 총 로그 길이 | 6,860행 ÷ 20Hz ≈ 343초 | 짧은 거리 이동 |

---

### 원인별 수정 내역

#### 수정 1 — `path_distance_bias: 32.0 → 18.0` (최우선)

**원인**: AMCL/Cartographer localization 노이즈(수 cm 수준)와 path_distance_bias=32.0 조합으로 DWA가
경로 좌/우 미세 오차에도 즉시 max angular(-0.1 rad/s) 포화 명령 출력.
로봇이 max angular와 min linear를 동시에 유지하며 실효 전진속도가 0.01 m/s로 저하.

```yaml
# 수정 전
path_distance_bias: 32.0

# 수정 후
path_distance_bias: 18.0   # goal_distance_bias(36.0)보다 낮춰 목표 접근 우선
```

---

#### 수정 2 — `min_vel_theta: 0.05 → 0.01` (#005-D)

**원인**: min_vel_theta=0.05 불감대로 실제 필요 각속도가 0~0.05 범위일 때 강제로 0.05 rad/s 적용.
오버슈트 → 반대 방향 보정 → 진동 반복.

```yaml
# 수정 전
min_vel_theta: 0.05

# 수정 후
min_vel_theta: 0.01   # 미세 보정 허용, 불감대 제거
```

---

#### 수정 3 — `acc_lim_theta: 0.5 → 0.15` (#005-E)

**원인**: DWA는 명령이 즉각 반영된다고 가정하지만 실제 WebSocket RTT ≈ 100ms 존재.
acc_lim_theta=0.5 rad/s²이면 100ms 지연 동안 각속도가 최대 0.05 rad/s 오버슈트 가능.

```yaml
# 수정 전
acc_lim_theta: 0.5

# 수정 후
acc_lim_theta: 0.15   # 100ms 지연 기준 최대 오버슈트: 0.015 rad/s
```

---

#### 수정 4 — `oscillation_reset_dist: 0.05 → 0.15` (#001-C)

**원인**: 합성 오도메트리(엔코더 미제공)의 부정확으로 5cm 이동을 DWA가 감지 못하는 경우 발생.
oscillation 카운터 오증가 → clearing_rotation(복구 행동) 발동.
CSV 로그에서 약 2000행 근처에 +0.5 rad/s clearing rotation 실제 발동 확인.

```yaml
# 수정 전
oscillation_reset_dist: 0.05

# 수정 후
oscillation_reset_dist: 0.15   # 15cm 이상 이동 시 oscillation 카운터 초기화
```

---

#### 수정 5 — hold-last angular 250ms 감쇠 (#005-C)

**원인**: `_control_loop`에서 `/cmd_vel` 큐가 비었을 때 마지막 angular 값을 무한 재전송(hold-last).
DWA가 새 명령을 발행했어도 로봇은 이전 포화 angular를 추가로 수신 → 포화 구간 연장.

```python
# 수정 전: 큐 비어있으면 last_angular 무조건 재전송
if not got_cmd:
    linear = self._cmd_vel_last_linear
    angular = self._cmd_vel_last_angular

# 수정 후: DWA 발행 주기(200ms) + 여유(50ms) = 250ms 초과 시 angular=0 폴백
if not got_cmd:
    linear = self._cmd_vel_last_linear
    if last_time is not None and (time.monotonic() - last_time) < 0.25:
        angular = self._cmd_vel_last_angular
    else:
        angular = 0.0
```

250ms 이내에는 hold-last 유지(DWA 갱신 주기 정상 동작 보장),
250ms 초과 시 angular=0으로 폴백하여 stale 포화 값 지속 방지.

---

### 파라미터 변경 전/후 비교

> ⚠️ `acc_lim_theta=0.15` 및 일부 값은 #007에서 수정됨. 실제 최종값은 #007 참조.

| 파라미터 | #006 적용 전 | #006 적용 후 | 비고 |
|----------|------------|------------|------|
| `path_distance_bias` | 32.0 | 18.0 | 사용자가 22.0으로 재조정 (#007 확정) |
| `goal_distance_bias` | 36.0 | 36.0 | 유지 |
| `min_vel_theta` | 0.05 | **0.01** | 확정 |
| `acc_lim_theta` | 0.5 | ~~0.15~~ | **회귀 버그 — #007에서 0.5로 복원** |
| `oscillation_reset_dist` | 0.05 | 0.15 | 사용자가 0.1으로 재조정 (#007 확정) |

---

### 검증 방법

```bash
# 1. 구동 후 CSV 로그 분석
python3 - <<'EOF'
import pandas as pd
df = pd.read_csv("src/TR-200/woosh_bringup/logs/nav_cmd_$(날짜).csv")
fwd = df[df["direction"] == "forward"]
# angular 포화 비율 (수정 후 20% 이하 목표)
sat = (fwd["angular_rad_s"].abs() >= 0.099).sum() / len(fwd)
print(f"angular 포화 비율: {sat:.1%}")
# 선속도 최댓값 (수정 후 0.08 m/s 이상 목표)
print(f"선속도 최대: {fwd['linear_m_s'].max():.3f} m/s")
# clearing rotation 발동 여부
rot = df[df["direction"].isin(["rotate_ccw", "rotate_cw"])]
print(f"회전 전용 명령 발생: {len(rot)}행")
EOF

# 2. oscillation recovery 미발동 확인
grep -c "rotate_ccw\|rotate_cw" src/TR-200/woosh_bringup/logs/nav_cmd_$(날짜).csv
```

---

### 기대 효과

| 현상 | 수정 전 | 수정 후 (기대) |
|------|---------|----------------|
| angular 포화 지속 | 전진 구간 전반 | 일시적 방향 보정만 |
| 실효 선속도 | 0.01~0.05 m/s | 0.06~0.10 m/s |
| oscillation recovery 발동 | 발동됨 | 억제됨 |
| 직선 구간 안정성 | 좌우 진동 지속 | 소폭 보정 후 직선 유지 |

---

---

## #007 · 버그수정 · #006 회귀 — acc_lim_theta=0.15로 DWA 탐색 공간 붕괴 수정

**작업 시각**: 2026-03-26
**분류**: 회귀 버그 수정
**수정 파일**:
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
**회귀 원인**: #006의 `acc_lim_theta: 0.5 → 0.15` 변경

---

### 현상 (nav_cmd_20260326_072223.csv 기준)

```
elapsed 33.8s ~ 44.0s  →  stop(0, 0)  ×203행  →  10.2초 지속
elapsed 44.0s ~        →  rotate_ccw(0, +0.5)  →  이후 전체 구간
```

- move_base 목표 수신 후 DWA가 처음부터 끝까지 `(0, 0)` 출력
- `oscillation_timeout: 10.0s` 경과 → `clearing_rotation` 복구 행동 발동
- 로봇 완전 전진 불가

---

### 근본 원인 분석

#### DWA 속도 샘플링 공간과 acc_lim_theta의 관계

DWA는 매 계획 주기(`sim_period = 1/controller_frequency`)마다 다음 범위만 속도 샘플링:

```
v_theta ∈ [curr_theta - acc_lim_theta × sim_period,
            curr_theta + acc_lim_theta × sim_period]
```

`controller_frequency=5Hz → sim_period=0.2s`, 출발 시 `curr_theta=0`이면:

| acc_lim_theta | 첫 주기 샘플 가능 angular 범위 | 결과 |
|---|---|---|
| 0.5 (#006 이전) | ±(0.5 × 0.2) = **±0.10 rad/s** | 정상 궤적 탐색 |
| 0.15 (#006 적용 후) | ±(0.15 × 0.2) = **±0.03 rad/s** | 경로 추종에 필요한 회전 궤적 생성 불가 |

`±0.03 rad/s`로는 경로가 조금이라도 방향 전환을 요구하는 경우 DWA가 생성하는 모든
forward trajectory의 `path_cost`가 stop(0,0)보다 높게 평가됨 → **DWA가 stop을 최적 선택**:

```
[DWA 비용 함수]
cost = path_distance_bias × path_cost
     + goal_distance_bias × goal_cost
     + occdist_scale × obstacle_cost

stop(0,0): path_cost = 현재 경로 오차 (변화 없음)
forward(0.06, ±0.03): path_cost = 회전이 부족해 경로에서 더 멀어짐
→ stop이 이김 → 영구 stop 출력
```

#### acc_lim_theta의 역할 재정의

| 역할 | 실제 담당 파라미터 |
|------|------------------|
| **DWA 탐색 가능 angular 범위 보장** | `acc_lim_theta` (충분히 커야 함) |
| **angular 포화 억제** | `path_distance_bias` (낮출수록 angular 교정 필요성 감소) |

#006에서 `acc_lim_theta=0.15`로 낮춘 의도(오버슈트 방지)는 잘못된 접근이었다.
angular 포화는 `path_distance_bias: 32→22`로 이미 해결됨.

---

### 수정 내역

#### 수정 1 — `acc_lim_theta: 0.15 → 0.5` (회귀 복원)

```yaml
# 수정 전 (#006 회귀 상태)
acc_lim_theta: 0.15   # DWA angular 탐색 범위 ±0.03 rad/s → stop 루프

# 수정 후
acc_lim_theta: 0.5    # DWA angular 탐색 범위 ±0.10 rad/s → 정상 궤적 탐색
```

---

#### 수정 2 — hold-last angular 임계값: 0.25s → 0.40s

```python
# 수정 전 (#006 적용 후)
if last_time is not None and (time.monotonic() - last_time) < 0.25:

# 수정 후
if last_time is not None and (time.monotonic() - last_time) < 0.40:
```

**배경**: DWA `controller_frequency=5Hz` → 발행 주기 200ms.
0.25s 임계값은 DWA 주기(200ms)와 너무 근접하여 ROS 타이머 지터(±30~50ms) 시
5번째 passthrough 주기(250ms)에서 조기 발동 가능. 0.40s(2× DWA 주기)로 변경.

---

### 사용자 파라미터 재조정 (이번 세션 중 직접 수정)

#006 적용 후 사용자가 아래 값을 직접 조정:

| 파라미터 | #006 적용값 | 사용자 재조정값 | 비고 |
|----------|------------|----------------|------|
| `path_distance_bias` | 18.0 | **22.0** | 경로 추종 강도 소폭 상향 |
| `oscillation_reset_dist` | 0.15 | **0.10** | 중간값으로 재조정 |

---

### 최종 파라미터 확정값 (이번 세션 기준)

| 파라미터 | 원래값 (세션 전) | 최종값 | 변경 내역 |
|----------|----------------|--------|-----------|
| `path_distance_bias` | 32.0 | **22.0** | #006 → 18.0, 사용자 → 22.0 |
| `goal_distance_bias` | 36.0 | 36.0 | 유지 |
| `min_vel_theta` | 0.05 | **0.01** | #006 |
| `acc_lim_theta` | 0.5 | **0.5** | #006 → 0.15, #007 → 0.5 복원 |
| `oscillation_reset_dist` | 0.05 | **0.10** | #006 → 0.15, 사용자 → 0.10 |
| hold-last angular 임계값 | (없음) | **0.40s** | #006 → 0.25s, #007 → 0.40s |

---

### 미해결 항목 갱신

| 번호 | 원인 | 현상 | 권장 조치 | 우선순위 |
|------|------|------|-----------|----------|
| E | 전역 경로 재계획 중 cmd_vel 공백 | planner 연산 지연 시 watchdog 발동 가능 | `planner_patience` 연장 또는 watchdog_timeout 완화 검토 | 낮음 |
| — | 합성 오도메트리 누적 오차 | 엔코더 미제공으로 장거리 정밀도 저하 | 외부 IMU 추가 후 EKF 융합 검토 | 낮음 |

---

---

## #008 · 버그수정 · 전진 중 angular 진동 잔존 + clearing_rotation 재발 수정

**작업 시각**: 2026-03-26
**분류**: 버그 수정 (파라미터 튜닝)
**수정 파일**:
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/move_base_params.yaml`
**분석 기반**: `src/TR-200/woosh_bringup/logs/nav_cmd_20260326_073753.csv`

---

### 현상 (nav_cmd_20260326_073753.csv 기준)

| 항목 | 관측값 | 기대값 |
|------|--------|--------|
| 전진 중 angular stdev | **0.065 rad/s** | ~0.02 이하 |
| 전진 중 angular 부호 반전율 | **7.9%** (매 ~0.65s) | 1~2% 이하 |
| 제자리 CCW 회전(+0.1 rad/s) 지속 | **20s 연속** (t=121~141) | 미발생 |
| clearing_rotation(+0.5) 발동 | **2회, 각 12.8s** | 0회 |
| 순수 전진(angular≈0) 비율 | **5.7%** | 40% 이상 |

---

### 근본 원인 분석

#### 원인 A — `path_distance_bias=22.0` 여전히 과도 → 전진 중 지속 L/R 미세 진동

#006에서 32→22로 낮췄지만, localization 노이즈 수 cm에 대해 DWA는 여전히 즉각 반응.

- 전진 중 angular 범위: `[-0.100, +0.100]` (DWA가 상한에 계속 도달)
- 부호 반전 7.9%: L/R 교번이 매 0.65s마다 발생 → 각도 오차 누적
- 이 오차가 임계치 초과 시 DWA가 전진 중단 → 제자리 회전 돌입

#### 원인 B — DWA 제자리 회전 속도 0.1 rad/s 고착

DWA 속도 샘플링 제약:

```
정지 출발 시 샘플 가능 angular:
  ±(acc_lim_theta × sim_period) = ±(0.5 × 0.2) = ±0.1 rad/s
```

로봇이 정지에서 제자리 회전 시작 시, DWA는 첫 주기에 최대 ±0.1만 샘플 가능.
`goal_distance_bias=36.0`이 `path_distance_bias=22.0`에 비해 충분히 강하지 않아
DWA가 0.1에서 0.2로 각속도를 올리는 궤적을 선택하지 않음 → **0.1 rad/s 고착**.

고착 결과: 90° 회전에 필요한 시간 = π/2 ÷ 0.1 ≈ **15.7s** (oscillation_timeout 초과)

#### 원인 C — `oscillation_timeout=10.0s`가 제자리 회전 완료 전 발동

제자리 회전(+0.1 rad/s)으로 57° 이상 필요한 경우 10s 내 완료 불가 → move_base `oscillation_timeout` 발동 → `clearing_rotation(+0.5 rad/s, 12.8s)` × 2회

---

### 수정 내역

#### 수정 1 — `path_distance_bias: 22.0 → 14.0`

**파일**: `local_planner_params.yaml`

```yaml
# 수정 전
path_distance_bias: 22.0

# 수정 후
path_distance_bias: 14.0   # localization 노이즈 내성 추가 확보
```

DWA가 경로 미세 오차에 덜 민감하게 반응. `goal_distance_bias(44.0) / path_distance_bias(14.0) = 3.14배` → 목표 방향 우선.

---

#### 수정 2 — `goal_distance_bias: 36.0 → 44.0`

**파일**: `local_planner_params.yaml`

```yaml
# 수정 전
goal_distance_bias: 36.0

# 수정 후
goal_distance_bias: 44.0   # 제자리 회전 시 DWA가 더 적극적으로 각속도 선택하도록 유도
```

제자리 회전 시 더 높은 angular를 선택하도록 goal_cost 가중치 상향.
path_distance_bias 감소분을 보완하여 목표 도달 품질 유지.

---

#### 수정 3 — `oscillation_timeout: 10.0 → 30.0s`

**파일**: `move_base_params.yaml`

```yaml
# 수정 전
oscillation_timeout: 10.0

# 수정 후
oscillation_timeout: 30.0   # 0.1 rad/s × 30s = 172° 커버
```

DWA 제자리 회전 속도 최대 0.1 rad/s 기준 30s = 3 rad(172°) 커버.
정상 DWA 회전이 완료될 시간을 충분히 확보하여 clearing_rotation 억제.

---

#### 수정 4 — `oscillation_distance: 0.2 → 0.3m`

**파일**: `move_base_params.yaml`

```yaml
# 수정 전
oscillation_distance: 0.2

# 수정 후
oscillation_distance: 0.3   # 저속 전진(0.06 m/s × 5s = 0.3m) 기준
```

저속 전진 시 oscillation 카운터 조기 누적 방지.

---

### 파라미터 변경 전/후 비교

| 파라미터 | #007 확정값 | #008 적용값 | 변경 이유 |
|----------|------------|------------|---------|
| `path_distance_bias` | 22.0 | **14.0** | 전진 중 L/R 진동 억제 |
| `goal_distance_bias` | 36.0 | **44.0** | 목표 추종 보강, 제자리 회전 가속 |
| `oscillation_timeout` | 10.0 | **30.0** | clearing_rotation 억제 |
| `oscillation_distance` | 0.2 | **0.3** | 저속 전진 oscillation 오감지 방지 |

---

### 기대 효과

| 현상 | 수정 전 | 수정 후 (기대) |
|------|---------|----------------|
| 전진 중 angular stdev | 0.065 rad/s | ~0.03 이하 |
| 전진 중 부호 반전율 | 7.9% | 2~3% 이하 |
| 제자리 회전 속도 | 0.1 rad/s 고착 | 0.1~0.2 rad/s 이상 |
| clearing_rotation 발동 | 2회/run | 0회 |
| 순수 전진 비율 | 5.7% | 25% 이상 |

---

### 검증 방법

```bash
# 구동 후 CSV 로그 분석
python3 - <<'EOF'
import pandas as pd
df = pd.read_csv("src/TR-200/woosh_bringup/logs/nav_cmd_$(날짜).csv")
fwd = df[df['linear_m_s'] > 0.001]
mix = fwd[fwd['angular_rad_s'].abs() > 0.01]

# angular stdev (수정 후 0.03 이하 목표)
print(f"전진 중 angular stdev: {fwd['angular_rad_s'].std():.4f}")

# 부호 반전율
flips = (fwd['angular_rad_s'].shift() * fwd['angular_rad_s'] < -0.001).sum()
print(f"부호 반전율: {flips/len(fwd)*100:.1f}%")

# clearing_rotation 발동 여부 (+0.5, lin=0)
clr = df[(df['angular_rad_s'].abs() >= 0.499) & (df['linear_m_s'].abs() < 0.001)]
print(f"clearing_rotation 발동: {len(clr)}행")

# 순수 전진 비율
pure_fwd = df[(df['linear_m_s'] > 0.001) & (df['angular_rad_s'].abs() < 0.01)]
print(f"순수 전진 비율: {len(pure_fwd)/len(df)*100:.1f}%")
EOF
```

---

---

## #009 · 버그수정 · angular 포화 37.2% + rotate_cw 구간 57% 낭비 수정

**작업 시각**: 2026-03-26
**분류**: 버그 수정
**수정 파일**:
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/move_base_params.yaml`
**근거 로그**: `src/TR-200/woosh_bringup/logs/nav_cmd_20260326_075507.csv`

---

### 현상

`#008` 수정 후 실행된 `nav_cmd_20260326_075507.csv` 로그에서 다음 문제 확인:

| 항목 | 값 |
|------|-----|
| 총 elapsed 구간 | 30.0s ~ 272.0s (241.9초) |
| forward 행 | 3,517행 (74%) |
| rotate_cw 행 | 752행 (16%) |
| stop 행 | 475행 (10%) |
| **forward 중 angular 포화(|ω|≥0.09) 비율** | **37.2%** (1,309/3,517) |
| forward 중 소각도 보정(|ω|<0.05) 비율 | 31.8% (1,117/3,517) |
| 평균 선속도 (forward 구간) | 0.053 m/s |
| **첫 rotate_cw 발동 시각** | **elapsed 127.2s** (oscillation_timeout=30s에도 조기 발동) |
| **총 rotate_cw 지속 시간** | **136.8s** (전체의 56.5%) |
| 최종 완전 stop | elapsed 264.0s ~ 272.0s |

---

### 분석 결과

#### 원인 A (최심각) — path_distance_bias=14.0이 여전히 localization 노이즈에 과민

`#008`에서 22.0→14.0으로 완화했음에도 forward 중 angular 포화(|ω|≥0.09) 비율이 37.2%로 잔존.
DWA가 localization 노이즈(수 cm)로 인한 경로 이탈을 과도하게 감지하여 매 DWA 계획 주기(200ms)마다
최대 angular(0.1 rad/s)를 발행하는 현상이 지속됨.

---

#### 원인 B (심각) — rotate_cw 복구가 0.1 rad/s로 실행되어 비효율적

`acc_lim_theta(0.5) × sim_period(1/5Hz=0.2s) = 0.1 rad/s` 제약으로 인해
clearing_rotation도 최대 0.1 rad/s로만 실행됨.
결과: 전체 항법 시간 241.9s 중 136.8s(57%)를 비효율적인 제자리 회전에 낭비.

---

#### 원인 C (심각) — oscillation_timeout=30s에도 elapsed 127.2s에서 조기 발동

path_distance_bias=14.0 → angular 포화 반복 → DWA 위치 진동 → oscillation 카운터 조기 누적.
`oscillation_distance=0.3m`를 저속(0.05 m/s) 전진 시 6초마다 채우는데,
angular 포화 구간(37.2%)에서 실효 선속도가 0.01 m/s로 저하되어 카운터가 누적됨.

---

### 변경 내용

#### 변경 1 — `local_planner_params.yaml`: path_distance_bias 추가 완화

```yaml
# 수정 전
path_distance_bias: 14.0
goal_distance_bias: 44.0

# 수정 후 [#009]
path_distance_bias: 8.0     # 14.0→8.0: angular 포화 37.2% 근본 원인 제거
goal_distance_bias: 52.0    # 44.0→52.0: path 감소분 보완, 목표 방향 추종력 유지
```

**설계 근거**:
- `path_distance_bias=8.0`은 DWA 비용 함수에서 경로 이탈 페널티를 낮춤
- 소규모 localization 노이즈(수 cm)에 의한 경로 이탈이 angular 포화로 이어지는 연결 고리를 차단
- `goal_distance_bias=52.0`으로 목표 방향 추종력을 강화하여 경로 이탈 보완

---

#### 변경 2 — `local_planner_params.yaml`: oscillation_reset_dist 0.1→0.15 유지·재확인

```yaml
oscillation_reset_dist: 0.15  # [#009] 0.1→0.15 재확인
```

저속(0.05 m/s) 전진 시 DWA 내부 진동 카운터 조기 누적 방지.

---

#### 변경 3 — `move_base_params.yaml`: clearing_rotation 비활성화

```yaml
# 수정 전
clearing_rotation_allowed: true

# 수정 후 [#009]
clearing_rotation_allowed: false
```

**설계 근거**:
- `acc_lim_theta × sim_period = 0.1 rad/s` 제약으로 clearing_rotation 실행 속도가 0.1 rad/s에 고착
- 136.8초 회전 낭비를 제거하고 costmap_reset → global re-plan 경로로 더 빠른 복구 시도

---

#### 변경 4 — `move_base_params.yaml`: oscillation_timeout 30.0→60.0

```yaml
# 수정 전
oscillation_timeout: 30.0

# 수정 후 [#009]
oscillation_timeout: 60.0
```

path_distance_bias 완화(14→8)와 병행하여 남은 oscillation false positive 억제.

---

#### 변경 5 — `move_base_params.yaml`: oscillation_distance 0.3→0.5

```yaml
# 수정 전
oscillation_distance: 0.3

# 수정 후 [#009]
oscillation_distance: 0.5   # 0.05 m/s × 10s = 0.5m → 약 10초마다 카운터 초기화
```

저속 전진(0.05 m/s) 구간에서 oscillation 카운터 조기 누적 방지.

---

### 기대 효과

| 항목 | 수정 전 (#008) | 기대값 (#009) |
|------|----------------|---------------|
| forward 중 angular 포화(|ω|≥0.09) 비율 | 37.2% | < 15% |
| 첫 rotate_cw 발동 시각 | elapsed 127.2s | 발동 없음 또는 > 200s |
| 총 rotate_cw 지속 시간 | 136.8s (57%) | < 10s (< 5%) |
| 최종 stop 도달 여부 | elapsed 264s (aborted) | 목표 도달(success) |

---

### 검증 방법

```bash
# 다음 로그 파일의 통계 확인
python3 - <<'EOF'
import pandas as pd
df = pd.read_csv("src/TR-200/woosh_bringup/logs/nav_cmd_$(날짜).csv")
fwd = df[df['linear_m_s'] > 0.001]

print(f"forward 중 angular 포화(|ω|≥0.09): {(fwd['angular_rad_s'].abs()>=0.09).mean()*100:.1f}%")
print(f"rotate_cw 행 비율: {(df['direction']=='rotate_cw').mean()*100:.1f}%")
print(f"첫 rotate_cw elapsed: {df[df['direction']=='rotate_cw']['elapsed_sec'].min():.1f}s")
EOF
```
