#!/usr/bin/env bash

set -euo pipefail

TARGET_INTERFACE="can2"
BITRATE="${1:-500000}"
RESTART_MS="${RESTART_MS:-100}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0 [bitrate]" >&2
  exit 1
fi

if ! [[ "${BITRATE}" =~ ^[0-9]+$ ]] || [[ "${BITRATE}" -le 0 ]]; then
  echo "Invalid bitrate: ${BITRATE}" >&2
  exit 1
fi

find_peak_interface() {
  local interface driver_path

  for driver_path in /sys/class/net/*/device/driver; do
    [[ -e "${driver_path}" ]] || continue
    if [[ "$(basename "$(readlink -f "${driver_path}")")" == "peak_usb" ]]; then
      interface="$(basename "$(dirname "$(dirname "${driver_path}")")")"
      printf '%s\n' "${interface}"
      return 0
    fi
  done

  return 1
}

SOURCE_INTERFACE="$(find_peak_interface || true)"
if [[ -z "${SOURCE_INTERFACE}" ]]; then
  echo "No PEAK PCAN-USB SocketCAN interface found (driver: peak_usb)." >&2
  exit 1
fi

if [[ "${SOURCE_INTERFACE}" != "${TARGET_INTERFACE}" ]]; then
  if ip link show "${TARGET_INTERFACE}" &>/dev/null; then
    echo "Cannot rename ${SOURCE_INTERFACE}: ${TARGET_INTERFACE} already exists." >&2
    exit 1
  fi

  ip link set "${SOURCE_INTERFACE}" down
  ip link set "${SOURCE_INTERFACE}" name "${TARGET_INTERFACE}"
fi

ip link set "${TARGET_INTERFACE}" down
ip link set "${TARGET_INTERFACE}" type can \
  bitrate "${BITRATE}" restart-ms "${RESTART_MS}"
ip link set "${TARGET_INTERFACE}" txqueuelen 1000
ip link set "${TARGET_INTERFACE}" up

echo "${TARGET_INTERFACE} is ready at ${BITRATE} bit/s."
ip -details link show "${TARGET_INTERFACE}"
echo
echo "Monitor frames with: candump ${TARGET_INTERFACE}"
