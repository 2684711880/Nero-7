#!/usr/bin/env bash
# 默认启动双臂 CAN；可指定接口，例如 bash can.sh can0。
set -euo pipefail

if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
  echo '用法: bash can.sh [can0|can1 ...]'
  echo '默认以 1000000 bit/s 启动 can0、can1。'
  exit 0
fi

if [[ $# -eq 0 ]]; then
  set -- can0 can1
fi

for interface in "$@"; do
  case "$interface" in
    can0|can1) ;;
    *) echo "不支持的接口：$interface（可选 can0、can1）" >&2; exit 2 ;;
  esac
  ip link show dev "$interface" >/dev/null
done

if [[ $EUID -eq 0 ]]; then
  privilege=()
else
  privilege=(sudo)
fi

for interface in "$@"; do
  "${privilege[@]}" ip link set dev "$interface" down
  "${privilege[@]}" ip link set dev "$interface" up type can bitrate 1000000
  ip -details -statistics link show dev "$interface"
done
