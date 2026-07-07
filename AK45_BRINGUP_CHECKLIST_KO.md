# AK45-36 KV80 Bring-up Checklist

## 전원 및 배선

- AK45는 24V 전원을 사용한다.
- 허용 전압은 16~28V이다.
- 48V 연결은 금지한다.
- AK70과 AK45 전원 레일은 분리한다.
- CAN 통신 기준 GND가 공통으로 연결되어 있는지 확인한다.
- CAN은 1Mbps로 설정한다.
- CANH/CANL은 twisted pair로 배선한다.
- 버스 양 끝에 120Ω 종단을 둔다.
- star 배선은 금지한다.
- 긴 stub 배선은 피한다.
- 전원선과 CAN선은 물리적으로 분리한다.

## 금지 항목

- 0xFE 하드웨어 zero 명령 금지.
- encoder calibrate 금지.
- servo permanent origin 금지.
- firmware update 금지.
- driver parameter write 금지.
- 원점은 software zero만 사용한다.

## 단일 엔코더 주의

- AK45 출력축은 10°마다 내부 엔코더 위상이 반복될 수 있다.
- 센서값만으로 부팅 절대 위치를 확인할 수 없다.
- 전원 인가 후 반드시 수평 정렬 확인이 필요하다.
- HOMED 상태는 현재 실행 세션에만 유효하다.
- 저장된 software zero가 있어도 프로세스 재시작 후 자동 HOMED로 간주하지 않는다.

## 첫 시험 순서

1. 0x006 단독 감지.
2. 0x006 software zero 저장.
3. 0x006 전원 재인가 비교 기록.
4. 0x006 수평 자세 확인.
5. 0x006 +3° 이동.
6. 0x006 -3° 이동.
7. 0x006 0° 복귀.
8. 0x00C도 동일 시험.
9. 0x006 + 0x00C 동시 시험.
10. AK70 2개 + AK45 2개 혼합 시험.
11. 4개, 6개, 12개 순서로 확장.

## CAN Health 해석

- CAN error counter 증가: 배선, 종단, EMI, GND 문제 가능성.
- RX dropped 증가: socket 또는 프로그램 수신 처리 문제 가능성.
- TX dropped 또는 ENOBUFS: 전송 pacing 또는 queue 문제 가능성.
- BUS-OFF 증가: 물리 계층 또는 ACK 문제 가능성.
- 모터만 재부팅: 전원 강하 가능성.
- CAN counter 정상인데 timing overrun: scheduler 또는 CPU 문제 가능성.
