#!/usr/bin/env bash
set -euo pipefail

# txqueuelen 1000을 무조건 사용하지 않는다.
# 기본 256에서 실제 통계를 확인한 후 조절한다.
# 너무 큰 queue는 오래된 제어 명령을 뒤늦게 전송할 수 있다.

sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen "${CAN_TX_QUEUE_LEN:-256}"
sudo ip link set can0 up

ip -details -statistics link show can0
