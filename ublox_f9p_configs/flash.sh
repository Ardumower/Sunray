#!/usr/bin/env bash

# 1. probe ublox GPS receiver
# 2. let user choose a firmware image
# 3. flash image (using wine or Python script)
# 4. probe again receiver

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/ubxfwupdate.py"
FIRMWARE_DIR="$SCRIPT_DIR/firmwares"
WINE_UPDATER="$SCRIPT_DIR/u-center/ubxfwupdate.exe"
WINE_FLASH_XML="$SCRIPT_DIR/u-center/flash.xml"

PORT="${PORT:-}"
FLASH_BAUD="${FLASH_BAUD:-9600:9600:115200}"
PROBE_BAUD="${PROBE_BAUD:-9600}"
VERBOSE="${VERBOSE:-1}"
FLASH_METHOD="${FLASH_METHOD:-script}"
WINEPREFIX_VALUE="${WINEPREFIX:-/home/$USER/.wine-ublox-test}"
WINEDEBUG_VALUE="${WINEDEBUG:--all}"

mapfile -t firmware_files < <(find "$FIRMWARE_DIR" -maxdepth 1 -type f -name '*.bin' | sort)

if [[ ${#firmware_files[@]} -eq 0 ]]; then
  echo "No firmware images found in $FIRMWARE_DIR" >&2
  exit 1
fi

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
if [[ -n "$PORT" && -e "$PORT" ]]; then
  default_port="$PORT"
else
  for path in "${serial_candidates[@]}"; do
    if [[ "$path" == /dev/serial/by-id/* ]] && [[ "$path" == *u-blox* || "$path" == *ublox* ]]; then
      default_port="$path"
      break
    fi
  done
  if [[ -z "$default_port" ]]; then
    default_port="${serial_candidates[0]}"
  fi
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
      selected_port="$default_port"
      break
    fi
    echo "Default port $default_port is not present." >&2
    continue
  fi
  if [[ "$port_choice" =~ ^[0-9]+$ ]] && (( port_choice >= 1 && port_choice <= ${#serial_candidates[@]} )); then
    selected_port="${serial_candidates[$((port_choice - 1))]}"
    break
  fi
  echo "Invalid selection." >&2
done

echo
echo "Probing $selected_port at $PROBE_BAUD"
echo
set +e
python3 "$PYTHON_SCRIPT" --probe -p "$selected_port" -b "$PROBE_BAUD" -v "$VERBOSE"
probe_status=$?
set -e
echo
if [[ $probe_status -ne 0 ]]; then
  echo "Probe failed on $selected_port." >&2
fi

#read -r -p "Continue to firmware selection? [y/N]: " continue_choice
#if [[ ! "$continue_choice" =~ ^[Yy]$ ]]; then
#  echo "Aborted."
#  exit 1
#fi

echo "USB NOTE: ublox f9p firmware flashing via USB has the potential risk that"
echo "the device gets not properly flashed, and in that case you need to"
echo "manually activate the f9p safeboot:"
echo " disconnect f9p USB, attach an UART->USB adapter:"
echo " 1. connect f9p UART RX1 -- adapter TX"
echo " 2. connect f9p UART TX1 -- adapter RX"
echo " 3. connect f9p GND -- adapter GND"
echo " 4. connect f9p 5V  -- adapter 5V"
echo " 5. connect f9p SAFEBOOT_N to f9p GND"
echo " 6. power-on f9p (f9p is now in safeboot)"
echo " 7. disconnect f9p SAFEBOOT_N"
echo " 8. re-run this script to flash via UART-USB adapter"
echo ""


echo "Available firmware images:"
for i in "${!firmware_files[@]}"; do
  printf "  %2d) %s\n" "$((i + 1))" "$(basename "${firmware_files[$i]}")"
done

choice=""
while true; do
  read -r -p "Select firmware image [1-${#firmware_files[@]}]: " choice
  if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#firmware_files[@]} )); then
    break
  fi
  echo "Invalid selection." >&2
done

image="${firmware_files[$((choice - 1))]}"

echo
echo "Available flash methods:"
echo "   1) Python (Linux Python flasher)"
echo "   2) wine   (official ubxfwupdate.exe via Wine)"

method_default_choice="1"
case "$FLASH_METHOD" in
  wine) method_default_choice="2" ;;
  script) method_default_choice="1" ;;
  *)
    echo "Unsupported FLASH_METHOD default '$FLASH_METHOD'; falling back to script." >&2
    FLASH_METHOD="script"
    ;;
esac

method_choice=""
while true; do
  read -r -p "Select flash method [1-2] (Enter for ${FLASH_METHOD}): " method_choice
  if [[ -z "$method_choice" ]]; then
    method_choice="$method_default_choice"
  fi
  if [[ "$method_choice" == "1" ]]; then
    FLASH_METHOD="script"
    break
  fi
  if [[ "$method_choice" == "2" ]]; then
    FLASH_METHOD="wine"
    break
  fi
  echo "Invalid selection." >&2
done

echo
echo "Flashing $(basename "$image")"
echo "Port: $selected_port"
echo "Flash baud: $FLASH_BAUD"
echo "Method: $FLASH_METHOD"
echo

if [[ "$FLASH_METHOD" == "script" ]]; then
  python3 "$PYTHON_SCRIPT" "$image" -p "$selected_port" -b "$FLASH_BAUD" -v "$VERBOSE"  --no-fis 1 -C 1 -s 1 -t 1
else
  if ! command -v wine >/dev/null 2>&1; then
    echo "wine is not installed or not in PATH." >&2
    exit 1
  fi
  if [[ ! -f "$WINE_UPDATER" ]]; then
    echo "Wine updater not found: $WINE_UPDATER" >&2
    exit 1
  fi
  if [[ ! -f "$WINE_FLASH_XML" ]]; then
    echo "Wine flash description not found: $WINE_FLASH_XML" >&2
    exit 1
  fi

  env WINEPREFIX="$WINEPREFIX_VALUE" WINEDEBUG="$WINEDEBUG_VALUE" \
    wine "$WINE_UPDATER" "$image" -F "$WINE_FLASH_XML" \
    -p "$selected_port" -b "$FLASH_BAUD" -v "$VERBOSE" \
    --no-fis 1 -C 1 -s 1 -t 1
fi

echo
read -r -p "Replug the device now, then press Enter to run --probe... " _
echo
echo "Running probe on $selected_port at $PROBE_BAUD"
echo

python3 "$PYTHON_SCRIPT" --probe -p "$selected_port" -b "$PROBE_BAUD" -v "$VERBOSE"
