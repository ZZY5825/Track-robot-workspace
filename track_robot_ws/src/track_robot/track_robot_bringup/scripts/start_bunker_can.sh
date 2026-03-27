#!/bin/bash
set -e

echo "[INFO] Resetting CAN interface can0..."

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

echo "[INFO] CAN status:"
ip -details link show can0
