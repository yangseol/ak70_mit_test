# AK70 / AK45 휴머노이드 실로봇 제어 사용 가이드

> 프로젝트 경로: `~/ak70_mit_test`
> 메인 실행 파일: `ak70_control_center_gui.py`
> CAN 인터페이스: `can0`
> CAN bitrate: `1 Mbps`
> 제어 방식: SocketCAN + MIT Mode + 소프트웨어 원점

---

## 1. 프로젝트 개요

이 프로젝트는 휴머노이드 하체의 12개 관절 모터를 Ubuntu에서 제어하기 위한 통합 프로젝트다.

- AK70 계열: 10개
- AK45 계열: 2개
- 오른쪽 다리: ID 1~6
- 왼쪽 다리: ID 7~12
- 메인 GUI: `ak70_control_center_gui.py`
- 실시간 제어기: `run_realtime_controller.py`
- GUI 실행 시 실시간 제어기는 자동으로 실행되므로 별도로 실행하지 않는다.
- 모터 내부 하드웨어 원점은 변경하지 않고 YAML에 저장된 소프트웨어 원점을 사용한다.

---

## 2. 모터 ID 및 관절 매핑

| ID | Joint | Motor |
|---:|---|---|
| 1 | `right_hip_pitch` | AK70 |
| 2 | `right_hip_roll` | AK70 |
| 3 | `right_hip_yaw` | AK70 |
| 4 | `right_knee` | AK70 |
| 5 | `right_ankle_pitch` | AK70 |
| 6 | `right_ankle_roll` | AK45 |
| 7 | `left_hip_pitch` | AK70 |
| 8 | `left_hip_roll` | AK70 |
| 9 | `left_hip_yaw` | AK70 |
| 10 | `left_knee` | AK70 |
| 11 | `left_ankle_pitch` | AK70 |
| 12 | `left_ankle_roll` | AK45 |

방향 설정 관련 현재 확정 사항:

- ID 9 `direction_sign: +1`
- ID 10 `direction_sign: -1`
- 실제 모터 방향과 UI 시각화 방향은 별개다.
- UI에서만 필요한 반전은 calibration이나 controller 값이 아니라 화면 좌표 계산에만 적용한다.

---

## 3. 주요 파일

| 파일 | 용도 |
|---|---|
| `ak70_control_center_gui.py` | 통합 GUI 실행 파일 |
| `run_realtime_controller.py` | 지속 실행 실시간 모터 제어기 |
| `ak_realtime_core.py` | 실시간 제어 핵심 |
| `realtime_ipc.py` | GUI와 controller 사이 IPC |
| `motor_profiles.py` | 모터별 프로필 |
| `motion_monitor.py` | 정면/측면 모션 모니터 |
| `test_motion_monitor.py` | 모션 모니터 회귀 테스트 |
| `motor_calibration.yaml` | AK70 소프트웨어 원점과 방향 |
| `ak45_motor_calibration.yaml` | AK45 소프트웨어 원점과 방향 |
| `humanoid_12dof_walk_cycle_20260706_090542.txt` | 로컬 보행 프리셋 데이터 |

---

## 4. 절대 주의사항

### 4.1 하드웨어 원점 명령 금지

다음 프레임은 절대 보내지 않는다.

```text
FF FF FF FF FF FF FF FE
```

`...FE`는 모터 내부 원점을 변경하는 하드웨어 set-zero 명령이다.

현재 프로젝트는 반드시 소프트웨어 원점을 사용한다.

```text
joint_rad = raw_pos_rad - raw_zero_pos_rad
```

원점은 다음 파일에 저장한다.

- AK70: `motor_calibration.yaml`
- AK45: `ak45_motor_calibration.yaml`

### 4.2 GUI와 controller를 중복 실행하지 않기

정상 사용 시 다음 파일만 실행한다.

```bash
python3 ak70_control_center_gui.py
```

다음 파일을 별도 터미널에서 동시에 실행하지 않는다.

```text
run_realtime_controller.py
send_realtime_targets.py
```

중복 controller가 남아 있으면 반복 송신, lease 충돌, bus fault, `Transmit buffer full` 등이 발생할 수 있다.

### 4.3 모터가 움직이는 동안 배선 조작 금지

다음 작업 전에는 반드시 torque를 해제하고 전원을 끈다.

- CAN 커넥터 분리
- 전원 커넥터 분리
- 허브 교체
- 모터 순서 변경
- 다리 자세를 크게 변경하면서 케이블을 당기는 작업

### 4.4 `Transmit buffer full` 상태에서 시작 버튼 반복 금지

오류가 발생하면 시작 버튼을 계속 누르지 않는다.

1. GUI 종료
2. controller 종료
3. 배선과 전원 확인
4. CAN 초기화
5. 통신 확인
6. GUI 재실행

---

## 5. 정상 사용 전 물리 점검

전원을 켜기 전에 확인한다.

- PCAN USB가 확실히 연결되어 있는가
- CAN-H / CAN-L이 뒤바뀌지 않았는가
- PCAN과 모터 전원의 GND가 공통인가
- 커넥터 핀이 하우징 뒤로 밀려나지 않았는가
- 관절을 움직일 때 케이블이 팽팽하게 당겨지지 않는가
- 허브 입력 및 출력 커넥터가 끝까지 삽입되어 있는가
- 노출된 전원선이나 CAN선이 단락되지 않았는가
- 다리가 갑자기 움직여도 주변과 충돌하지 않는가

---

## 6. 권장 전원 및 실행 순서

### 6.1 일반 시작 순서

1. 모터와 링크가 안전한 자세인지 확인
2. PCAN USB 연결
3. CAN 하네스 및 전원 하네스 확인
4. 모터 전원 ON
5. Ubuntu에서 CAN 상태 초기화
6. GUI 실행
7. `전체 모터 원클릭 시작`을 한 번만 누름
8. READY 모터 수와 각 모터 상태 확인
9. 작은 목표각으로 먼저 시험
10. 정상 확인 후 보행 또는 전체 동작 실행

### 6.2 전체 실행 상태 초기화 후 GUI 실행

아래 명령은 calibration과 프로젝트 코드를 지우지 않는다.
실행 중인 프로세스, IPC 임시 파일, controller fault latch와 CAN 실행 상태만 초기화한다.

```bash
pkill -9 -f ak70_control_center_gui.py 2>/dev/null
pkill -9 -f run_realtime_controller.py 2>/dev/null
pkill -9 -f send_realtime_targets.py 2>/dev/null
pkill -9 -f detect_ak70_motors_once.py 2>/dev/null
pkill -9 -f cangen 2>/dev/null
pkill -9 -f canplayer 2>/dev/null
pkill -9 -f candump 2>/dev/null

cd ~/ak70_mit_test

rm -f .ak_realtime_controller.sock
rm -f .ak_realtime_controller.lock
rm -f .ak_realtime_client_*.sock
rm -f .ak_realtime_probe_*.sock

sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

ip -details -statistics link show can0

python3 ak70_control_center_gui.py
```

정상 기준:

```text
can state ERROR-ACTIVE
berr-counter tx 0 rx 0
bitrate 1000000
qlen 1000
```

---

## 7. GUI 사용 순서

메인 GUI 탭:

1. `시작 / 원점`
2. `수동 / 모니터`
3. `Isaac Sim`

### 7.1 시작 / 원점

#### 전체 모터 원클릭 시작

- controller 시작
- 모터 감지
- MIT mode 진입
- 소프트웨어 원점 및 모터 프로필 로드
- READY 가능한 모터만 제어 대상으로 구성

버튼은 한 번만 누른다.

모든 12개가 연결되지 않아도 READY subset으로 동작할 수 있다.
단, 연결되지 않은 모터는 `NOT FOUND` 또는 `NOT READY`로 표시된다.

#### 현재 자세 전체 원점 저장

현재 기계 자세를 소프트웨어 0도로 저장할 때 사용한다.

주의:

- 로봇이 원하는 원점 자세에 정확히 놓였는지 확인
- 다리가 지면이나 지그에 의해 비틀리지 않았는지 확인
- 저장 직후 각 관절 표시값이 0도 부근인지 확인
- 실수로 누르지 않도록 주의
- 하드웨어 `...FE` 명령과는 무관하며 YAML 소프트웨어 원점만 갱신해야 한다

### 7.2 Kp / Kd 빠른 조절

현재 GUI의 gain은 좌우가 분리되어 있지 않고 모터 종류별 공통값이다.

- AK70 KP: ID 1~5, 7~11 READY 모터에 공통
- AK70 KD: ID 1~5, 7~11 READY 모터에 공통
- AK45 KP: ID 6, 12 READY 모터에 공통
- AK45 KD: ID 6, 12 READY 모터에 공통

즉 오른쪽과 왼쪽을 따로 조절하는 기능은 현재 없다.

주의:

- KP를 갑자기 크게 올리지 않는다.
- 처음에는 작은 목표각으로 진동과 충격을 확인한다.
- KD가 너무 낮으면 진동할 수 있고, 너무 높으면 움직임이 둔해질 수 있다.
- gain 변경 후 모터 온도와 소음을 확인한다.

### 7.3 수동 / 모니터

상단 주요 기능:

- `현재값 유지`: 현재 실제 자세를 target으로 설정하여 급격한 이동을 줄임
- `모든 목표 0°`: READY 모터 목표를 소프트웨어 원점 0도로 설정
- `수동 조작 열기/닫기`: 모터별 수동 slider 영역 표시/숨김

수동 조작 권장 순서:

1. 전체 모터 시작
2. READY 상태 확인
3. `현재값 유지`
4. 수동 조작 열기
5. 한 관절씩 작은 범위로 이동
6. Actual과 Target 오차 확인
7. 이상 소음, 진동, 반대 방향 움직임이 있으면 즉시 정지

---

## 8. 모션 모니터 읽는 법

### 8.1 기본 표시

- Actual: 실선
- Target: 점선
- READY 모터: 정상 색상
- NOT READY 모터: 회색 또는 N/A

상단에는 다음 정보가 표시된다.

- Source
- READY 수
- 최대 오차
- 최대 오차가 발생한 joint

### 8.2 정면 보기

정면 보기는 주로 좌우 움직임을 확인한다.

- hip roll
- ankle roll
- 좌우 벌어짐
- 몸 중심선과 좌우 균형

발의 상세 형상보다는 hip-knee-ankle 선과 roll 방향을 간단히 표시한다.

현재 UI 전용 보정:

- ID 2 `right_hip_roll`: 정면 그림 좌우 방향만 반전
- 실제 controller 값, 숫자 표시, calibration은 변경하지 않음

### 8.3 오른쪽 / 왼쪽 측면 보기

측면 보기는 오른쪽 다리와 왼쪽 다리를 별도 패널로 표시한다.

- 오른쪽 다리 측면 보기
- 왼쪽 다리 측면 보기
- 화면 오른쪽: FRONT
- 화면 왼쪽: BACK

각 패널은 자체 캔버스 중앙을 기준으로 그린다.

확인 관절:

- hip pitch
- knee
- ankle pitch

현재 UI 전용 보정:

- ID 10 `left_knee`: 측면 기구학은 변경하지 않음
- ID 10: 모터 상태 bar 방향만 반전
- ID 11 `left_ankle_pitch`: 왼쪽 측면 발목/발 그림 방향만 반전
- 위 보정은 실제 제어, calibration, 숫자 Actual/Target 값을 변경하지 않음

---

## 9. 로컬 보행 프리셋 사용

현재 로컬 보행 파일:

```text
humanoid_12dof_walk_cycle_20260706_090542.txt
```

기본 데이터:

- 샘플 수: 24
- sample interval: 약 0.1초
- 1 cycle: 약 2.380952초
- 단위: degree
- 첫 샘플 전환: 약 1초 quintic transition
- GUI target update: 50 Hz
- persistent controller: 100 Hz

관절 이름 매핑:

| 보행 파일 | 프로젝트 joint |
|---|---|
| `hip_f_joint` | `hip_pitch` |
| `hip_a_joint` | `hip_roll` |
| `hip_r_joint` | `hip_yaw` |
| `knee_joint` | `knee` |
| `ankle_f_joint` | `ankle_pitch` |
| `ankle_r_joint` | `ankle_roll` |

사용 순서:

1. 전체 모터 원클릭 시작
2. READY 모터 확인
3. 현재값 유지
4. 작은 gain과 안전 지지 상태 확인
5. 로컬 보행 파일 선택
6. 첫 자세 전환 확인
7. 1 cycle 또는 짧은 구간부터 실행
8. Actual/Target 오차와 모터 온도 확인
9. 이상 시 즉시 전체 torque 해제

READY subset을 지원하므로 일부 모터만 연결된 상태에서도 가능한 범위 안에서 동작할 수 있다.
다만 실제 보행 시험은 전체 하체가 안전하게 지지된 상태에서 진행한다.

---

## 10. Isaac Sim 탭

현재 PC에 ROS 2 환경이 없거나 `/opt/ros/humble`이 설치되지 않은 경우 GUI에 다음과 같이 표시될 수 있다.

```text
Isaac Sim ROS 2: 사용 불가
```

로컬 보행 프리셋과 수동 제어는 ROS 2 없이도 사용할 수 있다.

Isaac Sim 연동 전에는 다음을 별도 확인한다.

- ROS 2 배포판
- ROS_DOMAIN_ID
- ROS_LOCALHOST_ONLY
- topic 이름과 message type
- degree/radian 단위
- joint 이름 매핑
- 안전한 target limit

---

## 11. 정상 종료 순서

권장 종료 순서:

1. 보행 또는 수동 target 갱신 중지
2. `현재값 유지` 또는 안전한 자세로 복귀
3. GUI의 `전체 TORQUE 해제`
4. 모터가 힘을 해제했는지 확인
5. GUI 종료
6. controller 프로세스가 남았는지 확인
7. 모터 전원 OFF
8. 필요할 때 PCAN USB 분리

프로세스 확인:

```bash
pgrep -af "ak70|realtime_controller|cangen|canplayer|cansend|candump"
```

정상 종료 후 관련 프로세스가 남아 있으면 다음으로 종료한다.

```bash
pkill -9 -f ak70_control_center_gui.py 2>/dev/null
pkill -9 -f run_realtime_controller.py 2>/dev/null
pkill -9 -f send_realtime_targets.py 2>/dev/null
pkill -9 -f candump 2>/dev/null
```

---

## 12. AK70 통신 직접 확인

AK70 대상 ID:

```text
1, 2, 3, 4, 5, 7, 8, 9, 10, 11
```

ID 6과 12는 AK45이므로 AK70용 패킷만으로 정상 여부를 판정하지 않는다.

### 12.1 감시 터미널

```bash
cd ~/ak70_mit_test
candump -e -x can0 | tee all_motor_can_check.log
```

### 12.2 AK70 순차 검사

다른 터미널에서:

```bash
for n in 1 2 3 4 5 7 8 9 10 11
do
    id=$(printf "%03X" "$n")

    echo "========== TEST ID 0x$id =========="

    cansend can0 ${id}#FFFFFFFFFFFFFFFC
    sleep 0.15

    cansend can0 ${id}#8000800000000800
    sleep 0.35
done
```

프레임 의미:

```text
FFFFFFFFFFFFFFFC  → MIT mode 진입
8000800000000800  → zero torque
```

정상 예시:

```text
can0 TX ... 001 [8] FF FF FF FF FF FF FF FC
can0 RX ... 001 [8] ...

can0 TX ... 001 [8] 80 00 80 00 00 00 08 00
can0 RX ... 001 [8] ...
```

`TX`만 있고 해당 ID의 `RX`가 없으면 그 모터의 응답이 없는 것이다.

### 12.3 테스트 후 AK70 MIT mode 종료

```bash
for n in 1 2 3 4 5 7 8 9 10 11
do
    id=$(printf "%03X" "$n")
    cansend can0 ${id}#FFFFFFFFFFFFFFFD
    sleep 0.05
done
```

---

## 13. 오류별 대응

### 13.1 `Transmit buffer full`

의미:

- CAN 프레임이 정상적으로 처리되지 않음
- 송신 큐가 차고 있음
- 물리 버스 ACK가 없거나 controller가 중복 실행 중일 수 있음

확인 순서:

1. GUI와 controller 전부 종료
2. 배선 접촉 확인
3. 모터 전원 확인
4. PCAN과 GND 확인
5. CAN 초기화
6. 단독 `cansend` / `candump`로 RX 확인
7. RX가 확인된 뒤 GUI 실행

### 13.2 `cannot arm while bus fault is latched`

controller가 이전 bus fault를 기억하고 있어 재시작을 막는 상태다.

해결:

```bash
pkill -9 -f ak70_control_center_gui.py 2>/dev/null
pkill -9 -f run_realtime_controller.py 2>/dev/null

cd ~/ak70_mit_test
rm -f .ak_realtime_controller.sock
rm -f .ak_realtime_controller.lock
rm -f .ak_realtime_client_*.sock
rm -f .ak_realtime_probe_*.sock
```

그다음 CAN을 down/up하고 GUI를 다시 실행한다.

### 13.3 `Network is down`

`can0`가 내려간 상태다.

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

### 13.4 `No buffer space available [Error Code 105]`

대부분 `Transmit buffer full`과 같은 계열이다.

- ACK가 없는 버스
- 단선 또는 접촉 불량
- 중복 송신
- bus-off 반복
- 큐가 찬 상태

`txqueuelen 1000`은 오류 발생 시간을 늦출 수 있지만 근본 해결책은 아니다.

### 13.5 알 수 없는 CAN ID 수신

예:

```text
0x200
0x903
```

controller는 다음 프레임을 안전하게 무시해야 한다.

- unmanaged CAN ID
- extended frame
- error frame
- remote frame
- 허용 범위를 벗어난 arbitration ID

알 수 없는 ID 하나 때문에 controller 전체가 종료되면 안 된다.

---

## 14. CAN 통계 읽는 법

```bash
ip -details -statistics link show can0
```

현재 상태 판단에서 가장 중요한 항목:

```text
can state ERROR-ACTIVE
berr-counter tx 0 rx 0
```

다음 값은 누적 기록일 수 있다.

```text
re-started
error-pass
bus-off
RX packets
TX packets
dropped
```

예를 들어 `bus-off 43`이 보여도 현재 상태가 `ERROR-ACTIVE`, `tx 0 rx 0`이고 숫자가 더 증가하지 않으면 현재는 복구된 상태일 수 있다.

실시간 증가 확인:

```bash
watch -n 0.5 'ip -details -statistics link show can0'
```

판정:

```text
bus-off 43 유지      → 새 bus-off 없음
bus-off 43 → 44 증가 → 새로운 CAN 오류 발생
```

---

## 15. PCAN을 뽑지 않고 복구하는 방법

### 15.1 일반 소프트웨어 복구

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

### 15.2 BUS-OFF 수동 재시작

```bash
sudo ip link set can0 type can restart
```

### 15.3 PEAK USB 드라이버 재로드

프로세스를 모두 종료한 뒤:

```bash
sudo ip link set can0 down 2>/dev/null
sudo modprobe -r peak_usb
sudo modprobe peak_usb
```

이후 다시 설정:

```bash
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

주의:

- 다른 PEAK CAN 장치도 같이 재시작될 수 있다.
- 모듈이 사용 중이면 제거가 실패할 수 있다.
- 누적 통계까지 완전히 초기화되지 않으면 PCAN USB를 재연결한다.

---

## 16. 하드웨어 장애 분리 방법

같은 PCAN, 같은 직결 케이블, 같은 전원에서 모터만 교체한다.

```text
정상 모터 → RX 정상
특정 모터 → RX 없음
```

이 경우 의심 순서:

1. 해당 모터 CAN ID
2. MIT / Servo mode
3. CAN bitrate
4. CAN 포트 설정
5. 모터 커넥터 핀 접촉
6. 내부 CAN 트랜시버
7. 내부 드라이버 보드

초록 LED는 보통 전원과 MCU 부팅 상태를 나타내며 CAN 통신 성공을 보장하지 않는다.
CAN 성공은 Ubuntu에서 실제 `RX` 프레임으로 확인한다.

전원 OFF 상태에서 확인:

- CAN-H 연속성
- CAN-L 연속성
- GND 연속성
- CAN-H ↔ CAN-L 단락 여부
- CAN-H ↔ GND 단락 여부
- CAN-L ↔ GND 단락 여부
- 커넥터 핀이 뒤로 밀렸는지
- 관절 자세에 따라 케이블이 당겨지는지

---

## 17. 현재 UI 표시 보정 요약

현재 UI 표시 보정은 실제 모터 제어와 분리되어 있다.

| ID | 적용 위치 | 내용 |
|---:|---|---|
| 1 | 기존 UI 표시 경로 | 기존 화면 표시 보정 유지 |
| 2 | 정면 기구학 전용 | `right_hip_roll` 정면 좌우 방향 반전 |
| 10 | bar 전용 | `left_knee` 모터 상태 bar 방향 반전 |
| 11 | 왼쪽 측면 기구학 전용 | `left_ankle_pitch` 발목/발 그림 방향 반전 |

원칙:

- Actual/Target 원본값 변경 금지
- calibration 변경 금지
- direction_sign 변경 금지
- CAN/IPC payload 변경 금지
- 화면 좌표 계산에서 필요한 곳에 한 번만 적용
- 중복 sign 적용 금지

---

## 18. 코드 검증

모션 모니터와 GUI 수정 후:

```bash
cd ~/ak70_mit_test

python3 -m py_compile motion_monitor.py
python3 -m py_compile ak70_control_center_gui.py

python3 -m pytest -v -k "motion_monitor or visualization"

python3 - <<'PY'
import ak70_control_center_gui
import motion_monitor
print("GUI and motion monitor import OK")
PY
```

최근 확인된 결과:

```text
31 passed, 151 deselected
GUI and motion monitor import OK
```

전체 테스트가 필요하면:

```bash
python3 -m pytest -v
```

---

## 19. GitHub에 올리기 전 확인

```bash
cd ~/ak70_mit_test

git status
git diff --stat
git diff -- motion_monitor.py
git diff -- ak70_control_center_gui.py
git diff -- test_motion_monitor.py
```

커밋에 포함하면 안 되는 항목:

- `.ak_realtime_controller.sock`
- `.ak_realtime_controller.lock`
- `.ak_realtime_client_*.sock`
- `.ak_realtime_probe_*.sock`
- `__pycache__/`
- `.pytest_cache/`
- 일시적인 CAN 로그
- 비밀번호, 토큰, 개인 키

권장 `.gitignore` 항목:

```gitignore
__pycache__/
*.py[cod]
.pytest_cache/
.ak_realtime_controller.sock
.ak_realtime_controller.lock
.ak_realtime_client_*.sock
.ak_realtime_probe_*.sock
all_motor_can_check.log
```

---

## 20. 권장 Git 커밋

문서와 UI 수정 파일을 확인한 뒤:

```bash
cd ~/ak70_mit_test

git add \
  ak70_control_center_gui.py \
  motion_monitor.py \
  test_motion_monitor.py \
  docs/AK_REAL_ROBOT_CONTROL_GUIDE_KO.md \
  README.md \
  .gitignore

git status

git commit -m "Document real robot workflow and refine motion monitor UI"

git push origin main
```

`git status`에서 의도하지 않은 calibration 변경이나 임시 파일이 포함되지 않았는지 반드시 확인한 다음 커밋한다.

---

## 21. 빠른 시작 요약

```text
1. 배선·전원·기구 안전 확인
2. 모터 전원 ON
3. 전체 실행 상태 초기화
4. can0 ERROR-ACTIVE / tx 0 rx 0 확인
5. ak70_control_center_gui.py 실행
6. 전체 모터 원클릭 시작 1회
7. READY 상태 확인
8. 현재값 유지
9. 작은 수동 동작 시험
10. gain과 Actual/Target 확인
11. 보행 또는 전체 동작 실행
12. 종료 시 전체 TORQUE 해제
13. GUI 종료
14. 모터 전원 OFF
```
