# AK70-10 모터 관리 프로젝트

이 저장소는 T-Motor AK70-10 모터를 SocketCAN(`can0`, 1 Mbps)으로 감지하고, MIT mode 패킷으로 상태를 읽고, software zero 기준으로 캘리브레이션 및 원점 이동을 보조하는 프로젝트입니다.

중요한 점은 이 프로젝트가 모터 내부 hardware zero를 바꾸는 방식이 아니라는 것입니다. `motor_calibration.yaml`에 저장된 software zero 값을 사용하며, 관절각은 아래 계산식으로 구합니다.

```text
joint_rad = raw_pos_rad - raw_zero_pos_rad
```

## 주요 파일

- `ak70_motor_manager_gui.py`: Tkinter 기반 GUI 관리 앱입니다. CAN 확인, CAN 설정, 모터 감지, 원점 저장, 계획 확인, 원점 이동 helper 실행을 한 화면에서 수행합니다.
- `detect_multi_motors_once.py`: `0x001`부터 `0x00A`까지 모터를 한 번 스캔하고, 감지 여부와 calibration 여부를 출력합니다.
- `read_multi_joints_once.py`: 여러 모터에 zero-torque command를 1회씩 보내고 software offset이 적용된 joint state를 출력합니다.
- `plan_multi_zero_move.py`: 모터를 움직이지 않고 현재 joint angle 기준으로 원점 이동 가능 여부를 판정합니다.
- `nudge_multi_joints_once.py`: 계획 확인 후 `READY_FOR_LIMITED_NUDGE` 모터만 순차적으로 제한 nudge합니다.
- `capture_motor_zero_once.py`: 단일 모터의 현재 raw position을 software zero로 저장합니다.
- `calibration.py`: `motor_calibration.yaml` 로드 및 software offset 계산 함수가 있습니다.
- `mit_packet.py`: MIT command 패킷 생성과 feedback 후보 파싱 함수가 있습니다.
- `motor_calibration.yaml`: 모터별 software zero 값이 저장되는 파일입니다.

## GUI 실행 방법

```bash
cd ~/ak70_mit_test
python3 ak70_motor_manager_gui.py
```

GUI 실행만으로는 CAN command를 보내지 않습니다. 사용자가 버튼을 눌렀을 때만 기존 helper script가 실행됩니다.

## CAN 설정 방법

GUI에서 `CAN 설정` 버튼을 누르면 `can0`를 1 Mbps로 설정합니다. 내부적으로 아래 명령을 `pkexec`로 실행하려고 시도합니다.

```bash
ip link set can0 down
ip link set can0 type can bitrate 1000000
ip link set can0 up
```

권한 문제로 실패하면 GUI 로그창에 직접 실행할 명령이 표시됩니다. `CAN 확인` 버튼은 `ip -details link show can0` 결과만 확인하며 모터 CAN command를 보내지 않습니다.

## 모터 감지 방법

터미널에서 직접 확인할 때:

```bash
python3 detect_multi_motors_once.py --channel can0 --yes
```

필요할 때만 MIT enter packet을 먼저 보내려면:

```bash
python3 detect_multi_motors_once.py --channel can0 --enter-mit --yes
```

GUI에서는 `모터 감지` 버튼을 누르면 기본 범위 `0x001~0x00A`를 스캔하고 표에 감지 여부, calibration 여부, raw/joint 값을 반영합니다.

## 원점 저장 방법

원점 저장은 현재 위치를 `motor_calibration.yaml`의 software zero로 저장하는 기능입니다. 모터 내부 zero를 바꾸지 않으며 `0xFE` set-zero 명령을 보내지 않습니다.

GUI에서는 감지된 모터가 정확히 1개일 때만 `원점 저장` 버튼이 활성화됩니다. 저장 전 확인창이 두 번 뜨고, 저장 전에 `backups/` 폴더에 `motor_calibration_YYYYMMDD_HHMMSS.yaml` 형식의 백업을 만듭니다.

터미널에서 직접 저장할 때:

```bash
python3 capture_motor_zero_once.py --channel can0 --motor-id 0x005
```

스크립트 안내에 따라 `YES`, `SAVE`를 입력해야 실제 YAML 파일이 수정됩니다.

## 전체 원점이동 방법

먼저 계획 확인으로 모든 모터가 안전 범위인지 확인합니다.

```bash
python3 plan_multi_zero_move.py --channel can0 --motor-ids 0x005,0x007,0x00A --max-start-error-rad 0.9
```

제한 nudge 원점 이동은 아래처럼 실행합니다.

```bash
python3 nudge_multi_joints_once.py --channel can0 --motor-ids 0x005,0x007,0x00A --max-start-error-rad 0.9 --kp 2.0 --pulses 10
```

GUI에서는 `전체 계획확인`으로 상태를 확인한 뒤, calibration이 있는 감지 모터에 대해 `전체 원점이동`을 실행합니다. 실제 position command는 GUI가 직접 만들지 않고, 검증된 `nudge_multi_joints_once.py`를 subprocess로 호출합니다.

## 안전 주의사항

- `0xFE` set-zero 명령은 사용하지 않습니다.
- 기본 calibration 방식은 hardware zero가 아니라 software zero입니다.
- `motor_calibration.yaml` 수정 전에는 백업을 만들어야 합니다.
- 여러 모터를 동시에 직접 위치제어하지 않습니다.
- 원점 이동은 `nudge_multi_joints_once.py`의 interlock 판정을 통과한 경우에만 제한 pulse로 순차 실행합니다.
- `BLOCKED_TOO_FAR`, `NO_FEEDBACK`, `NO_CALIBRATION`이 있으면 자동 원점 이동을 진행하지 않습니다.
- 로봇에 장착된 상태에서는 벤치탑보다 낮은 gain과 작은 시작 오차 기준을 사용해야 합니다.
- GUI에는 원하는 각도를 입력해서 이동하는 기능이 없습니다.
