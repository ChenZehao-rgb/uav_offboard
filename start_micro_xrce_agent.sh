#!/usr/bin/env bash

set -Eeuo pipefail

readonly BAUD_RATE=921600

show_help() {
    cat <<'EOF'
用法：
  ./start_micro_xrce_agent.sh [设备]

设备参数示例：
  ACM0             -> /dev/ttyACM0（默认）
  ACM1             -> /dev/ttyACM1
  ttyUSB0          -> /dev/ttyUSB0
  /dev/ttyPixhawk  -> /dev/ttyPixhawk

示例：
  ./start_micro_xrce_agent.sh
  ./start_micro_xrce_agent.sh ACM1
  ./start_micro_xrce_agent.sh /dev/ttyACM1
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    show_help
    exit 0
fi

device_arg="${1:-ACM0}"

case "$device_arg" in
    /dev/*)
        device="$device_arg"
        ;;
    tty*)
        device="/dev/$device_arg"
        ;;
    *)
        device="/dev/tty$device_arg"
        ;;
esac

if [[ ! -e "$device" ]]; then
    echo "错误：串口设备不存在：$device" >&2
    echo "请检查设备名称，或执行 ls /dev/ttyACM* /dev/ttyUSB* 查看可用串口。" >&2
    exit 1
fi

if [[ ! -c "$device" ]]; then
    echo "错误：目标不是字符设备：$device" >&2
    exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    echo "错误：未找到 MicroXRCEAgent，请先确认它已安装并位于 PATH 中。" >&2
    exit 1
fi

echo "设置串口权限：$device"
sudo chmod 777 -- "$device"

echo "启动 MicroXRCEAgent：$device，波特率：$BAUD_RATE"
exec MicroXRCEAgent serial --dev "$device" -b "$BAUD_RATE"