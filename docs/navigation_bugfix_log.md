# Navigation 버그 수정 로그

**대상 로봇**: Woosh TR-200 (차동 구동, ROS1 Noetic)
**작업자**: LDJ @ KATECH

---

## 2026-03-26

---

# 1. 가다 서다(Stop-and-Go) 모션 원인 분석

**작업 일시**: 2026-03-26
**분석 파일**:
- `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`
- `src/TR-200/woosh_bringup/scripts/cmd_vel_adapter.py`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/move_base_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_planner_params.yaml`
- `src/TR-200/woosh_navigation/MoveBase/woosh_navigation_mb/config/local_costmap_params.yaml`
- `docs/navigation_algorithms.md`

---

## 1-1. 현상

`move_base_on` 모드로 자율주행 시 로봇이 등속도로 주행하지 않고 **가다 서다를 반복**하는 불규칙 모션 발생.

---

## 1-2. 원인 분석 결과

### 1-2-1. (최심각) SmoothTwistController + CmdVelAdapter 이중 WebSocket 연결 충돌

`move_base_on` 모드에서 **두 개의 독립 WebSocket 연결**이 동시에 로봇에 twist_req를 전송하는 구조적 충돌.

| 노드 | identity | WebSocket 역할 |
|------|----------|----------------|
| `SmoothTwistController` (woosh_service_driver.py) | `twist_ctrl` | `/mobile_move` 서비스 처리 — 항상 연결 유지 |
| `CmdVelAdapter` (cmd_vel_adapter.py, 서브프로세스) | `cmd_vel_adapter` | move_base `/cmd_vel` 전달 — move_base_on 시 추가 연결 |

**충돌 메커니즘**:
- `main()` 에서 `Thread(target=_run_asyncio, daemon=True).start()` 로 SmoothTwistController가 WebSocket 연결을 선점한 상태에서
- `launcher.start_cmd_vel_adapter()` 가 별도 서브프로세스로 두 번째 WebSocket 연결을 시도
- Woosh SDK 내부에서 중복 연결 처리로 명령 드롭 또는 연결 재설정 발생 → **순간 정지 반복**

```
[수정 전 구조]
SmoothTwistController  → WebSocket #1 (항상 연결, 유휴 루프 유지)
cmd_vel_adapter (subprocess) → WebSocket #2 (충돌 발생)
```

---

### 1-2-2. (심각) await twist_req 블로킹으로 실질 제어 주기 저하

`cmd_vel_adapter.py`의 `_send_twist` 는 `await self.robot.twist_req(...)` 이므로 WebSocket 왕복 시간(RTT ~100ms) 동안 루프가 블로킹된다.

| 파라미터 | 설정값 | 실제 동작 |
|----------|--------|-----------|
| `control_hz` (cmd_vel_adapter) | 20 Hz → 주기 50ms | twist_req RTT ~100ms → 실제 주기 100~150ms |
| `controller_frequency` (move_base) | 5 Hz → cmd_vel 간격 200ms | 200ms 안에 어댑터가 1회 전송도 불확실 |

결과: 이동 중 속도 명령이 간헐적으로 누락 → stop-and-go.

---

### 1-2-3. (중간) DWA oscillation 복구 행동 반복 트리거

합성 오도메트리(엔코더 없음, SDK `PoseSpeed.twist` dt 적분) 부정확성과 WebSocket 지연이 복합 작용.

```yaml
# local_planner_params.yaml
oscillation_reset_dist: 0.05  # DWA 내부 진동 판단 기준: 5cm

# move_base_params.yaml
oscillation_timeout: 10.0     # 10초 이상 진동 감지 시 복구
oscillation_distance: 0.2     # 0.2m 이동해야 진동 카운터 초기화
clearing_rotation_allowed: true  # 복구 시 제자리 회전 허용
```

오도메트리가 느리게 업데이트되면 DWA가 5cm 이동을 감지 못해 oscillation 카운터 증가 → **복구 행동(제자리 회전 + 재계획) 발동** → 정지 후 재출발 반복.

---

### 1-2-4. (중간) path_distance_bias 과도 설정으로 잦은 경로 교정

```yaml
# local_planner_params.yaml
path_distance_bias: 32.0  # 전역 경로 추종 강도 (높을수록 경로에 강하게 달라붙음)
goal_distance_bias: 24.0
occdist_scale: 0.02
```

WebSocket 지연으로 로봇이 경로에서 조금이라도 이탈하면 DWA가 급격한 방향 수정 트라젝토리를 반복 선택 → 속도 감속이 주기적으로 발생.

---

### 1-2-5. (경미) 전역 경로 재계획 중 cmd_vel 공백 → watchdog 발동 가능성

```yaml
# move_base_params.yaml
planner_frequency: 1.0  # Hz — 전역 경로 재계획 1초마다
```

재계획 연산이 1초를 초과하면 cmd_vel 발행이 중단되어 `watchdog_timeout: 1.0s` 발동 → 강제 정지.

---

## 1-3. 원인 요약

```
최우선 수정 대상:
  [1] SmoothTwistController + CmdVelAdapter 동시 WebSocket 연결 → 명령 충돌/드롭
  [2] await twist_req (RTT ~100ms) + control_hz 20Hz → 실질 전송 주기 저하

구조적 원인 (추후 튜닝):
  [3] 합성 오도메트리 부정확 → DWA oscillation 오감지 → 복구 행동 반복
  [4] path_distance_bias: 32.0 → 잦은 교정 동작 유발
```

---

# 2. 원인 1 수정 — 이중 WebSocket 연결 충돌 해결

**작업 일시**: 2026-03-26
**수정 파일**: `src/TR-200/woosh_bringup/scripts/woosh_service_driver.py`

---

## 2-1. 해결 방향

`cmd_vel_adapter.py` 서브프로세스를 `move_base_on` 모드에서 기동하지 않고, `SmoothTwistController` 내부에 `/cmd_vel` 패스스루 기능을 통합한다.

```
[수정 후 구조]
SmoothTwistController  → WebSocket #1 (단일 연결)
  ├─ /mobile_move 명령 → quintic 프로파일 전송  (우선순위 높음)
  └─ /cmd_vel 수신    → 동일 WebSocket으로 20Hz 패스스루
```

---

## 2-2. 변경 내용

### 2-2-1. import 추가

```python
# 수정 전
from geometry_msgs.msg import PoseWithCovarianceStamped

# 수정 후
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist as RosTwist
```

ROS `geometry_msgs/Twist`를 woosh proto `Twist`와 이름 충돌 없이 사용하기 위해 `RosTwist` 별칭으로 import.

---

### 2-2-2. SmoothTwistController.__init__ — cmd_vel 상태 변수 추가

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

### 2-2-3. _send_twist — angular 파라미터 추가

```python
# 수정 전
async def _send_twist(self, linear=0.0):
    await self.robot.twist_req(Twist(linear=linear, angular=0.0), ...)

# 수정 후
async def _send_twist(self, linear=0.0, angular=0.0):
    await self.robot.twist_req(Twist(linear=linear, angular=angular), ...)
```

기존 호출부는 모두 positional/keyword 방식으로 호환됨 (하위 호환 유지).

---

### 2-2-4. 신규 메서드 — _cmd_vel_callback

```python
def _cmd_vel_callback(self, msg):
    """move_base가 발행하는 /cmd_vel 을 수신해 큐에 넣는다."""
    linear = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
    angular = max(-0.5, min(0.5, float(msg.angular.z)))
    with self._cmd_vel_lock:
        self._cmd_vel_last_time = time.monotonic()
        self._cmd_vel_last_linear = linear
        self._cmd_vel_last_angular = angular
    # 큐가 꽉 찼으면 기존 값 버리고 최신으로 교체
    try:
        self._cmd_vel_queue.get_nowait()
    except Empty:
        pass
    try:
        self._cmd_vel_queue.put_nowait((linear, angular))
    except Exception:
        pass
```

- 속도 클리핑: 선속도 `±max_speed(0.12 m/s)`, 각속도 `±0.5 rad/s`
- `Queue(maxsize=1)` + 교체 패턴으로 항상 최신 명령만 유지

---

### 2-2-5. 신규 메서드 — enable_cmd_vel_passthrough

```python
def enable_cmd_vel_passthrough(self, watchdog_timeout=1.0):
    """cmd_vel 패스스루 모드를 활성화한다 (move_base_on 전용)."""
    self._cmd_vel_watchdog_timeout = watchdog_timeout
    self._cmd_vel_sub = rospy.Subscriber(
        "/cmd_vel", RosTwist, self._cmd_vel_callback, queue_size=1
    )
    self._cmd_vel_enabled = True
```

`main()` 에서 `launcher.start_cmd_vel_adapter()` 대신 이 메서드를 호출하여 WebSocket 이중 연결을 방지.

---

### 2-2-6. _control_loop — cmd_vel 패스스루 루프 추가

```python
async def _control_loop(self):
    period_idle = 0.01          # /mobile_move 대기 루프 간격
    period_cmdvel = 1.0 / 20.0  # cmd_vel 패스스루 루프 간격 (20 Hz)
    _watchdog_fired = False

    while True:
        # ── /mobile_move 거리 명령 우선 처리 (선점) ──────────────────
        try:
            distance = self.command_queue.get_nowait()
            success, msg = await self._move_exact_distance(distance)
            self.result_queue.put((success, msg))
            continue
        except Empty:
            pass

        # ── cmd_vel 패스스루 (move_base_on 모드) ─────────────────────
        if not self._cmd_vel_enabled:
            await asyncio.sleep(period_idle)
            continue

        # 큐에서 최신 명령 꺼내기, 없으면 마지막 명령 유지 (hold-last)
        ...

        # watchdog: 1.0초 이상 /cmd_vel 미수신 시 자동 정지
        elapsed = time.monotonic() - last_time
        if elapsed >= self._cmd_vel_watchdog_timeout:
            linear, angular = 0.0, 0.0

        await self._send_twist(linear, angular)
        await asyncio.sleep(period_cmdvel)
```

**우선순위 설계**:
1. `/mobile_move` 거리 명령 — quintic 프로파일로 정밀 이동 (선점)
2. `/cmd_vel` 패스스루 — move_base DWA 출력 20Hz 전달 (대기 중)

두 경로가 asyncio 단일 루프에서 직렬 실행되므로 동시 전송 충돌 없음.

---

### 2-2-7. main() — start_cmd_vel_adapter 제거

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

## 2-3. 검증 방법

### 런타임 확인

```bash
# 1. WebSocket 연결 수 확인 — ESTABLISHED가 1개여야 정상
ss -tnp | grep 5480

# 2. ROS 노드 목록 — cmd_vel_adapter 노드가 없어야 정상
rosnode list

# 3. cmd_vel 패스스루 활성화 로그 확인
rostopic echo /rosout | grep "cmd_vel 패스스루 활성화"
```

### 동작 확인

```bash
# move_base로 목표 전달 후 /cmd_vel 토픽 수신 및 로봇 주행 확인
rostopic echo /cmd_vel
```

---

## 2-4. 미해결 항목 (추후 튜닝 권장)

| 항목 | 현상 | 권장 조치 |
|------|------|-----------|
| DWA oscillation_reset_dist | 5cm 기준으로 오감지 빈발 | `0.10~0.15m` 로 상향 조정 |
| path_distance_bias | 32.0으로 설정되어 급격한 교정 동작 | `20.0~24.0` 으로 완화 |
| controller_frequency | 5Hz — WebSocket RTT 대비 낮음 | 현 설정 유지 또는 `planner_patience` 연장 검토 |
| 합성 오도메트리 누적 오차 | 엔코더 미제공으로 장거리 정밀도 저하 | 외부 IMU 추가 후 EKF 융합 검토 |
