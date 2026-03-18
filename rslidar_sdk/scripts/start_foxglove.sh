#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${SDK_DIR}/config/config.yaml"
NODE_BIN="${1:-${SDK_DIR}/build/rslidar_sdk_node}"

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "config not found: ${CONFIG_PATH}" >&2
  exit 1
fi

if [[ ! -x "${NODE_BIN}" ]]; then
  echo "node binary not executable: ${NODE_BIN}" >&2
  echo "usage: $0 /absolute/path/to/rslidar_sdk_node" >&2
  exit 1
fi

cleanup() {
  set +e
  if [[ -n "${NODE_PID:-}" ]]; then
    kill "${NODE_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

"${NODE_BIN}" "${CONFIG_PATH}" &
NODE_PID=$!
echo "rslidar_sdk_node pid=${NODE_PID}"

wait "${NODE_PID}"
