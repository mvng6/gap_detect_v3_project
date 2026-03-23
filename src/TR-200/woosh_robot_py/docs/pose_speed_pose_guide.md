# PoseSpeed.pose 완전 이해 가이드

> 이 문서는 Woosh TR-200 SDK의 `PoseSpeed.pose` 필드가 무엇인지, 어떻게 받아오고, 어디에 쓸 수 있는지를 **초등학생도 이해할 수 있는 수준**으로 설명합니다.

---

## 1. pose 란 무엇인가?

### 쉬운 비유: 체육관 바닥의 로봇

체육관 바닥에 격자 무늬 타일이 깔려 있다고 상상해보세요.

```
         ↑  Y (앞)
         |
         |
  -------+------→ X (오른쪽)
         |
         |
        (0,0) = 출발점
```

로봇이 이 체육관 안에서 움직이면, 로봇은 항상 자신의 위치를 세 가지 숫자로 알고 있습니다:

| 숫자 | 이름 | 의미 |
|------|------|------|
| `x` | X 좌표 | 출발점에서 **오른쪽**으로 몇 미터 |
| `y` | Y 좌표 | 출발점에서 **앞쪽**으로 몇 미터 |
| `theta` | 방향각 | 로봇이 **어느 방향**을 보고 있는지 (라디안, 단위: rad) |

이 세 숫자 묶음을 **포즈(Pose)** 라고 부릅니다.

---

## 2. theta(방향각) 읽는 법

`theta` 는 **라디안(radian)** 단위입니다. 라디안이 낯설다면 아래 표를 참고하세요:

```
      90° = π/2 ≈ 1.57 rad
           ↑
           |
180° ------+------ 0° = 0 rad  (오른쪽 = 기본 방향)
  ≈ 3.14   |
           ↓
      270° = -π/2 ≈ -1.57 rad
```

- `theta = 0.0` → 로봇이 오른쪽(+X)을 바라봄
- `theta = 1.57` → 로봇이 앞쪽(+Y)을 바라봄
- `theta = 3.14` → 로봇이 왼쪽(-X)을 바라봄
- `theta = -1.57` → 로봇이 뒤쪽(-Y)을 바라봄

---

## 3. PoseSpeed 전체 구조

`PoseSpeed`는 로봇이 "지금 어디에 있고, 얼마나 빠르게 움직이는가"를 담은 패킷입니다.

```
PoseSpeed
├── pose         ← 위치 + 방향 (이 문서의 주인공)
│   ├── x        : float  (단위: 미터)
│   ├── y        : float  (단위: 미터)
│   └── theta    : float  (단위: 라디안)
│
├── twist        ← 현재 속도
│   ├── linear   : float  (단위: m/s, 전진 속도)
│   └── angular  : float  (단위: rad/s, 회전 속도)
│
├── map_id       : int    (현재 사용 중인 지도 번호, 0=지도 없음)
└── mileage      : int    (로봇 생산 이후 누적 주행 거리, 단위: mm 추정)
```

### pose vs twist 차이

| | `pose` | `twist` |
|---|---|---|
| **뜻** | 지금 **어디**에 있나 | 지금 **얼마나 빠르게** 움직이나 |
| **비유** | 지도 위의 내 위치 | 자동차 속도계 |
| **값 유지** | 멈춰있어도 마지막 위치 유지 | 멈추면 0 이 됨 |

---

## 4. pose 의 출처: 로봇 내부 SLAM

`PoseSpeed.pose` 는 단순한 바퀴 회전 계산이 **아닙니다**.

```
 LiDAR 레이저 스캔
      │
      ▼
 로봇 내부 SLAM 엔진  ←── 지도(map)
      │
      ▼
 PoseSpeed.pose   ←── 이게 바로 로봇이 스스로 추정한 위치
```

**쉽게 말하면:** 로봇이 LiDAR로 주변 벽과 장애물을 스캔하고, 저장된 지도와 비교해서 "나는 지금 지도 위 이 위치에 있다"라고 계산한 결과입니다.

> ⚠️ **주의:** `map_id == 0` 이면 지도가 로드되지 않은 상태로, `pose.x`, `pose.y`, `pose.theta` 값은 **의미 없는 쓰레기 값**일 수 있습니다.

### 엄밀히 말하면 `/odom` 과는 다릅니다

`PoseSpeed.pose` 는 **지도(map) 기준 절대 위치에 가까운 값**입니다.
반면 ROS의 `/odom` 은 보통 **연속적으로 적분되는 상대 위치 추정**을 뜻합니다.

즉, 둘 다 `(x, y, theta)` 를 담을 수는 있지만 의미가 다릅니다:

| 항목 | `PoseSpeed.pose` | ROS `/odom` |
|------|------------------|-------------|
| 기준 | 로봇 내부 지도 좌표계 | 오도메트리 좌표계 |
| 맵 필요 여부 | 필요 (`map_id != 0`) | 보통 불필요 |
| 값 성격 | LiDAR + 지도 매칭 결과 | 속도/엔코더 적분 결과 |
| 특징 | 재로컬라이즈 시 값이 바뀔 수 있음 | 가능한 한 연속적으로 이어져야 함 |

따라서 `PoseSpeed.pose` 를 곧바로 "엄밀한 의미의 `/odom`" 이라고 부르면 혼동이 생길 수 있습니다.
더 정확한 표현은 **"로봇 내부 localization 결과"** 입니다.

---

## 5. 입력: 어떻게 데이터를 받아오나?

두 가지 방법이 있습니다.

### 방법 A: 한 번만 요청하기 (Request)

"지금 위치 알려줘" 하고 한 번 묻는 방식입니다.

```python
from woosh.proto.robot.robot_pb2 import PoseSpeed
from woosh_interface import NO_PRINT

# 한 번만 요청
pose_speed, ok, msg = await robot.robot_pose_speed_req(
    PoseSpeed(), NO_PRINT, NO_PRINT
)

if ok:
    pose = pose_speed.pose
    print(f"현재 위치: x={pose.x:.2f}m, y={pose.y:.2f}m")
    print(f"현재 방향: theta={pose.theta:.2f}rad")
    print(f"지도 ID: {pose_speed.map_id}")
```

### 방법 B: 계속 받기 (Subscribe)

로봇이 움직일 때마다 자동으로 위치를 알려주는 방식입니다.

```python
from woosh.proto.robot.robot_pb2 import PoseSpeed
from woosh_interface import NO_PRINT

def on_pose_updated(pose_speed: PoseSpeed):
    """로봇 위치가 바뀔 때마다 자동으로 호출됨"""
    pose = pose_speed.pose

    # 지도가 로드된 상태일 때만 유효
    if pose_speed.map_id != 0:
        print(f"[업데이트] x={pose.x:.2f}, y={pose.y:.2f}, θ={pose.theta:.2f}")
    else:
        print("[경고] 지도 없음 — pose 값 신뢰 불가")

# 구독 시작 (이후 위치가 바뀌면 on_pose_updated 자동 호출)
await robot.robot_pose_speed_sub(on_pose_updated, NO_PRINT)
```

---

## 6. 출력: 어디에 활용되나?

### 활용처 A: ROS `Odometry` 메시지로 내보내기 (현재 미구현)

현재 `woosh_sensor_bridge.py` 는 `twist`를 적분해서 위치를 계산합니다.
하지만 `pose` 를 직접 사용하면 로봇 내부 추정값을 ROS 쪽으로 전달할 수 있습니다.

> ⚠️ 다만 이것을 곧바로 **엄밀한 의미의 `/odom`** 이라고 보면 안 됩니다.
> `PoseSpeed.pose` 는 지도 기반 위치이므로, TF 의미상으로는 `/odom` 보다는
> `map` 기준 pose 또는 별도 디버그/상태 토픽에 더 가깝습니다.

```
PoseSpeed.pose  →  nav_msgs/Odometry  →  별도 pose/odometry 토픽
                       (x, y, theta
                       → quaternion 변환)
```

### 활용처 B: RViz 로봇 위치 시각화 (현재 구현됨)

`woosh_rviz_debug.py` 에서 이미 `pose` 를 사용해 로봇을 지도 위에 그립니다.

```python
# woosh_rviz_debug.py:358-362
pose = pose_speed.pose
self._append_trace_point(pose.x, pose.y, pose.theta)

# woosh_rviz_debug.py:767-771
msg.pose.position.x = self.latest_pose.pose.x
msg.pose.position.y = self.latest_pose.pose.y
qz, qw = self._yaw_to_quaternion(self.latest_pose.pose.theta)
```

### 활용처 C: 로봇 초기화 시 현재 위치 전달 (현재 구현됨)

`woosh_service_driver.py` 에서 로봇을 초기화할 때 `pose` 를 그대로 전달합니다.

```python
# woosh_service_driver.py:173-175
init_robot.pose.x     = pose_speed.pose.x
init_robot.pose.y     = pose_speed.pose.y
init_robot.pose.theta = pose_speed.pose.theta
```

### 활용처 D: 목적지 도달 확인

로봇에게 "A 지점으로 가라" 명령을 내린 후, 실제로 도착했는지 확인할 수 있습니다.

```python
import math

TARGET_X = 3.0   # 목표 x (미터)
TARGET_Y = 2.0   # 목표 y (미터)
THRESHOLD = 0.1  # 10cm 이내면 도착으로 간주

def is_arrived(pose_speed: PoseSpeed) -> bool:
    dx = pose_speed.pose.x - TARGET_X
    dy = pose_speed.pose.y - TARGET_Y
    distance = math.sqrt(dx**2 + dy**2)
    return distance < THRESHOLD
```

---

## 7. 전체 데이터 흐름 그림

```
┌──────────────────────────────────────────────────────┐
│              Woosh TR-200 로봇 내부                    │
│                                                      │
│  LiDAR 스캔 + 지도  →  내부 SLAM  →  pose (x,y,θ)   │
│  바퀴 모터          →  내부 추정  →  twist (v, ω)    │
│  주행 거리 누적      →            →  mileage         │
└──────────────────────────────┬───────────────────────┘
                               │  WebSocket (protobuf)
                               ▼
┌──────────────────────────────────────────────────────┐
│           woosh_robot_py (Python SDK)                │
│                                                      │
│  robot_pose_speed_req()  →  PoseSpeed 한 번 반환     │
│  robot_pose_speed_sub()  →  PoseSpeed 계속 수신      │
└──────────────────────────────┬───────────────────────┘
                               │
              ┌────────────────┼──────────────────┐
              ▼                ▼                  ▼
     woosh_sensor_bridge  woosh_rviz_debug  woosh_service_driver
     (현재: twist 적분)   (pose 시각화)     (초기화 시 pose 전달)
     (개선여지: pose 직접 사용)
```

---

## 8. 현재 코드의 한계와 개선 방향

### 현재 상태 (`woosh_sensor_bridge.py`)

```python
# 현재: twist 를 시간 적분해서 위치 계산
self.odom_x += linear * math.cos(self.odom_theta) * dt
self.odom_y += linear * math.sin(self.odom_theta) * dt
self.odom_theta += angular * dt
```

**문제:** 적분은 시간이 지날수록 오차가 쌓입니다.
(= 조금씩 틀리다가 나중엔 크게 틀림)

### 개선 방향

`PoseSpeed.pose` 를 직접 사용하면 오차 누적 없이 로봇 내부 추정값을 그대로 활용할 수 있습니다.
다만 이것은 **정확한 pose 소스 활용**이지, **엄밀한 의미의 `/odom` 대체**와는 조금 다릅니다.

권장 해석은 다음과 같습니다:

- `PoseSpeed.twist` 적분 → 연속적인 `/odom`
- `PoseSpeed.pose` → 지도 기반 현재 위치(`/amcl_pose` 와 비슷한 역할의 값)

> **단, 전제 조건:** `map_id != 0` (지도가 로드된 상태) 일 때만 유효.
> 지도 없이 SLAM 없이 움직이는 상황에서는 `pose` 값을 신뢰할 수 없으므로 `twist` 적분이 유일한 대안입니다.

---

## 9. 자주 묻는 질문

**Q. `pose` 와 오도메트리(`/odom`) 는 같은 건가요?**
A. 아닙니다. 둘 다 위치를 나타내지만 **엄밀히는 다른 종류의 값**입니다. `/odom` 은 보통 속도/엔코더 적분으로 이어지는 연속적인 오도메트리이고, `PoseSpeed.pose` 는 LiDAR와 지도를 비교해 얻은 **내부 localization 결과**입니다. 따라서 `PoseSpeed.pose` 는 지도 의존적이며, 재로컬라이즈되면 값이 순간적으로 바뀔 수도 있습니다.

**Q. `pose` 값이 항상 정확한가요?**
A. 아닙니다. LiDAR 기반 SLAM이므로 복도처럼 반복적인 구조에서는 위치 추정에 실패(납치 문제)할 수 있습니다. `map_id` 가 0인 경우, 또는 로봇이 갑자기 들려서 옮겨진 경우 값을 신뢰할 수 없습니다.

**Q. `mileage` 는 어디에 쓰나요?**
A. 로봇 전체 주행 거리 누적값으로, 주행 거리 기반 유지보수 주기 계산이나 테스트 로그 기록에 활용할 수 있습니다. 오도메트리 소스로는 적합하지 않습니다.

**Q. `theta` 의 단위가 왜 도(°)가 아니라 라디안인가요?**
A. ROS 표준이 라디안을 사용하고, 수학 계산(삼각함수)도 라디안 기준이기 때문입니다. 도로 변환하려면 `math.degrees(theta)` 를 사용하세요.

---

## 10. 관련 파일 위치

| 파일 | 역할 |
|------|------|
| [woosh/proto/robot/robot_pb2.pyi](../woosh/proto/robot/robot_pb2.pyi) | `PoseSpeed` 메시지 타입 정의 |
| [woosh/proto/util/common_pb2.pyi](../woosh/proto/util/common_pb2.pyi) | `Pose2D`, `Twist` 타입 정의 |
| [woosh_interface.py](../woosh_interface.py) | `robot_pose_speed_req`, `robot_pose_speed_sub` API |
| [examples/demo_lite.py](../examples/demo_lite.py) | `pose` 요청/구독 예제 코드 |
| [examples/monitor.py](../examples/monitor.py) | `pose` + `mileage` 출력 예제 |
| [../../woosh_bringup/scripts/woosh_rviz_debug.py](../../woosh_bringup/scripts/woosh_rviz_debug.py) | `pose` 를 RViz 시각화에 활용 |
| [../../woosh_navigation/AMCL/scripts/woosh_sensor_bridge.py](../../woosh_navigation/AMCL/scripts/woosh_sensor_bridge.py) | 현재 `twist` 적분 사용 (개선 여지 있음) |
