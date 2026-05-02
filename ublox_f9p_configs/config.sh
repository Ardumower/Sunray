#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/ubxconfig.py"
PROBE_SCRIPT="$SCRIPT_DIR/ubxfwupdate.py"
CONFIG_DIR="$SCRIPT_DIR/configs"

PORT="${PORT:-}"
AUTOBAUD="${AUTOBAUD:-1}"
PROBE_BAUD="${PROBE_BAUD:-9600}"
BAUD_SWEEP_LIST="${BAUD_SWEEP_LIST:-}"
LAYER="${LAYER:-all}"
ACK_TIMEOUT="${ACK_TIMEOUT:-2.0}"
INTER_PACKET_DELAY="${INTER_PACKET_DELAY:-0.1}"
VERBOSE="${VERBOSE:-1}"

mapfile -t config_files < <(find "$CONFIG_DIR" -maxdepth 1 -type f | sort)

if [[ ${#config_files[@]} -eq 0 ]]; then
  echo "No config files found in $CONFIG_DIR" >&2
  exit 1
fi

if [[ -z "$PORT" ]]; then
  declare -a serial_candidates=()
  while IFS= read -r path; do
    serial_candidates+=("$path")
  done < <(
    {
      find /dev/serial/by-id -maxdepth 1 -type l 2>/dev/null
      find /dev -maxdepth 1 -type c \( -name 'ttyUSB*' -o -name 'ttyACM*' \) 2>/dev/null
    } | sort -u
  )

  if [[ ${#serial_candidates[@]} -eq 0 ]]; then
    echo "No serial device paths found." >&2
    exit 1
  fi

  default_port=""
  for path in "${serial_candidates[@]}"; do
    if [[ "$path" == /dev/serial/by-id/* ]] && [[ "$path" == *u-blox* || "$path" == *ublox* ]]; then
      default_port="$path"
      break
    fi
  done
  if [[ -z "$default_port" ]]; then
    default_port="${serial_candidates[0]}"
  fi

  echo "Available serial device paths:"
  for i in "${!serial_candidates[@]}"; do
    printf "  %2d) %s\n" "$((i + 1))" "${serial_candidates[$i]}"
  done

  port_choice=""
  while true; do
    read -r -p "Select serial path [1-${#serial_candidates[@]}] (Enter for ${default_port}): " port_choice
    if [[ -z "$port_choice" ]]; then
      if [[ -e "$default_port" ]]; then
        PORT="$default_port"
        break
      fi
      echo "Default port $default_port is not present." >&2
      continue
    fi
    if [[ "$port_choice" =~ ^[0-9]+$ ]] && (( port_choice >= 1 && port_choice <= ${#serial_candidates[@]} )); then
      PORT="${serial_candidates[$((port_choice - 1))]}"
      break
    fi
    echo "Invalid selection." >&2
  done
fi

if [[ "$AUTOBAUD" != "0" ]]; then
  echo
  echo "Probing baud rate on $PORT"
  echo
  probe_cmd=(python3 "$PROBE_SCRIPT" --probe-baud-sweep -p "$PORT" -v "$VERBOSE")
  if [[ -n "$BAUD_SWEEP_LIST" ]]; then
    probe_cmd+=(--baud-sweep-list "$BAUD_SWEEP_LIST")
  fi
  set +e
  probe_output="$("${probe_cmd[@]}" 2>&1)"
  probe_status=$?
  set -e
  printf '%s\n' "$probe_output"
  if [[ $probe_status -ne 0 ]]; then
    echo "Failed to detect receiver baud on $PORT." >&2
    echo "Set PROBE_BAUD manually and run with AUTOBAUD=0 to bypass probing." >&2
    exit 1
  fi
  detected_baud="$(printf '%s\n' "$probe_output" | sed -n 's/^[[:space:]]*detected baud: \([0-9][0-9]*\)$/\1/p' | tail -n 1)"
  if [[ -z "$detected_baud" ]]; then
    echo "Probe succeeded but no detected baud was parsed from output." >&2
    echo "Set PROBE_BAUD manually and run with AUTOBAUD=0 to bypass probing." >&2
    exit 1
  fi
  PROBE_BAUD="$detected_baud"
fi

echo
echo "Probing receiver on $PORT at $PROBE_BAUD"
echo
python3 "$PROBE_SCRIPT" --probe -p "$PORT" -b "$PROBE_BAUD" -v "$VERBOSE"

echo "Available configs:"
for i in "${!config_files[@]}"; do
  printf "  %2d) %s\n" "$((i + 1))" "$(basename "${config_files[$i]}")"
done

config_choice=""
selected_config=""
while true; do
  read -r -p "Select config to apply [1-${#config_files[@]}]: " config_choice
  if [[ "$config_choice" =~ ^[0-9]+$ ]] && (( config_choice >= 1 && config_choice <= ${#config_files[@]} )); then
    selected_config="${config_files[$((config_choice - 1))]}"
    break
  fi
  echo "Invalid selection." >&2
done

echo
echo "Applying config $(basename "$selected_config")"
echo "Port: $PORT"
echo "Baud: $PROBE_BAUD"
echo "Layer: $LAYER"
echo

python3 "$PYTHON_SCRIPT" "$selected_config" \
  -p "$PORT" \
  -b "$PROBE_BAUD" \
  --layer "$LAYER" \
  --ack-timeout "$ACK_TIMEOUT" \
  --inter-packet-delay "$INTER_PACKET_DELAY"
