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

## 미해결 항목

> 분석에서 확인된 원인 중 아직 수정되지 않은 항목. 추후 작업 시 이 목록을 참조한다.

| 번호 | 원인 | 현상 | 권장 조치 | 우선순위 |
|------|------|------|-----------|----------|
| C | DWA oscillation_reset_dist 오감지 | 합성 오도메트리 부정확으로 oscillation 카운터 오증가 → 복구 행동 반복 | `oscillation_reset_dist: 0.05` → `0.10~0.15m`로 상향 | 중간 |
| D | path_distance_bias 과도 설정 | 경로 이탈 시 급격한 방향 교정 → 속도 감속 반복 | `path_distance_bias: 32.0` → `20.0~24.0`으로 완화 | 중간 |
| E | 전역 경로 재계획 중 cmd_vel 공백 | planner 연산 지연 시 watchdog 발동 가능 | `planner_patience` 연장 또는 watchdog_timeout 완화 검토 | 낮음 |
| — | 합성 오도메트리 누적 오차 | 엔코더 미제공으로 장거리 정밀도 저하 | 외부 IMU 추가 후 EKF 융합 검토 | 낮음 |
