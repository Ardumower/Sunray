#!/bin/bash

# start/stop sunray service
# start/stop camera streaming web service on "localhost:8081"

# Resolve paths independently of the caller's working directory.
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
cd "$SCRIPT_DIR" || exit 1
SIM_ROOT=$(dirname -- "$SCRIPT_DIR")
SIM_UNITS=(sunray-sim3d sunray-sim3d-world sunray-sim3d-display)

simulator_build() (
  set -eu
  case "$(uname -s):$(uname -m)" in
    Linux:aarch64|Linux:arm64) ;;
    *) echo "ERROR: build this simulator directly on the ARM64 Orange Pi (Linux)."; exit 1 ;;
  esac
  if systemctl is-active --quiet sunray-sim3d; then
    echo "Stop Sunray before building: sudo systemctl stop sunray-sim3d"
    echo "The world can remain running to preserve its position."
    exit 1
  fi
  for tool in cmake c++; do
    command -v "$tool" >/dev/null || { echo "Missing $tool; install build-essential and cmake."; exit 1; }
  done
  echo "Building ARM64 Cloud simulator: config_sim_ws.h, with RKNN grass inference."
  echo "Dependencies: build-essential cmake libbluetooth-dev libssl-dev libjpeg-dev, RKNN headers/runtime."
  if [ -z "${RKNN_INCLUDE_DIR:-}" ] && [ -f "$SIM_GPU_DIR/rknn/include/rknn_api.h" ]; then
    RKNN_INCLUDE_DIR="$SIM_GPU_DIR/rknn/include"
    RKNN_LIBRARY="${RKNN_LIBRARY:-$SIM_GPU_DIR/rknn/lib/librknnrt.so}"
  fi
  if [ -z "${RKNN_INCLUDE_DIR:-}" ] && [ -f "$HOME/rknn_model_zoo/3rdparty/rknpu2/include/rknn_api.h" ]; then
    RKNN_INCLUDE_DIR="$HOME/rknn_model_zoo/3rdparty/rknpu2/include"
  fi
  args=(-S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build-sim3d"
    -D "CONFIG_FILE=$SCRIPT_DIR/config_sim_ws.h" -D SUNRAY_REQUIRE_RKNN=ON)
  if [ -n "${RKNN_INCLUDE_DIR:-}" ]; then args+=(-D "RKNN_INCLUDE_DIR=$RKNN_INCLUDE_DIR"); fi
  if [ -n "${RKNN_LIBRARY:-}" ]; then args+=(-D "RKNN_LIBRARY=$RKNN_LIBRARY"); fi
  cmake "${args[@]}"
  cmake --build "$SCRIPT_DIR/build-sim3d" -j "${BUILD_JOBS:-4}"
  echo "Build complete. Start the simulator separately when ready."
)

simulator_install() (
  set -eu
  local staging
  staging=$(mktemp -d)
  trap 'rm -rf "$staging"' EXIT
  python3 - "$SIM_ROOT" "$SIM_GPU_DIR" "$(id -un)" "$staging" <<'PYUNITS'
import pathlib, re, sys
root, gpu, user, staging = sys.argv[1:]
if not all(re.fullmatch(r'[A-Za-z0-9_./-]+', s) for s in (root, gpu, user)):
    raise SystemExit('ERROR: unsupported characters in service paths or user name')
libs = ':'.join([gpu+'/mali/usr/lib/aarch64-linux-gnu/mali', gpu+'/mali/usr/lib/aarch64-linux-gnu', gpu+'/rga/usr/lib/aarch64-linux-gnu', gpu+'/rknn/lib'])
for p in (pathlib.Path(root)/'tools/sim3d/systemd').glob('*.service'):
    text = p.read_text().replace('/home/orangepi/Sunray-sim3d', root).replace('/home/orangepi/godot-camera-test', gpu).replace('User=orangepi', 'User='+user)
    text = text.replace('SupplementaryGroups=render\n', '')
    text = text.replace('[Service]\n', '[Service]\nSupplementaryGroups=render video\nEnvironment=LD_LIBRARY_PATH='+libs+'\n')
    (pathlib.Path(staging)/p.name).write_text(text)
PYUNITS
  sudo install -m 644 "$staging"/*.service /etc/systemd/system/
  sudo systemctl daemon-reload
)

# Pinned, checksum-verified vendor components; extracted beside the application.
SIM_GPU_DIR="$HOME/godot-camera-test"

simulator_download() {
  local url=$1 checksum=$2 target=$3
  if [ -f "$target" ] && echo "$checksum  $target" | sha256sum -c - >/dev/null 2>&1; then return; fi
  curl --fail --location --retry 3 --connect-timeout 15 --max-time 600 "$url" -o "$target.part" || return
  echo "$checksum  $target.part" | sha256sum -c - || return
  mv "$target.part" "$target"
}

simulator_setup() (
  set -euo pipefail
  case "$(uname -s):$(uname -m)" in
    Linux:aarch64|Linux:arm64) ;;
    *) echo "ERROR: setup requires an ARM64 Orange Pi with Linux."; exit 1 ;;
  esac
  # This driver bundle matches the vendor kernel, not mainline/Panfrost images.
  . /etc/os-release
  if [ "$ID:${VERSION_ID:-}" != ubuntu:22.04 ] || [[ $(uname -r) != 6.1.* ]] ||
     ! tr '\0' '\n' < /proc/device-tree/compatible | grep -q 'rockchip,rk3588' ||
     [ ! -c /dev/mali0 ]; then
    echo "ERROR: supported setup is Orange Pi RK3588(S), Ubuntu 22.04, vendor kernel 6.1 with /dev/mali0."
    echo "Install the matching Orange Pi OS image first; this tool does not replace kernels or device trees."
    exit 1
  fi
  if ! grep -qs '^connected$' /sys/class/drm/*HDMI*/status; then
    echo "ERROR: connect an HDMI dummy or monitor before setup."; exit 1
  fi
  if systemctl is-active --quiet sunray; then
    echo "ERROR: stop the regular sunray service before setting up the simulator."; exit 1
  fi
  # systemd ExecStart and Environment paths below intentionally require simple paths.
  case "$SIM_ROOT:$SIM_GPU_DIR" in
    *[!a-zA-Z0-9_./:-]*) echo "ERROR: checkout and home paths must not contain spaces or special characters."; exit 1 ;;
  esac
  echo "Setting up simulator, GPU display and NPU. The virtual world will restart at its spawn pose."
  sudo true
  for unit in "${SIM_UNITS[@]}"; do
    if [ "$(systemctl show "$unit" -p LoadState --value)" = loaded ]; then sudo systemctl stop "$unit"; fi
  done
  trap 'echo "ERROR: setup failed. Simulator remains stopped; correct the error and rerun setup."; sudo systemctl stop sunray-sim3d sunray-sim3d-world sunray-sim3d-display >/dev/null 2>&1 || true' ERR
  sudo apt-get update
  sudo env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l apt-get install -y --no-remove --no-upgrade -o Dpkg::Options::=--force-confold build-essential cmake libbluetooth-dev libssl-dev libjpeg-dev \
    curl ca-certificates unzip python3 xauth x11-xserver-utils xserver-common xkb-data \
    libegl1 libgl1 libgbm1 libaudit1 libbsd0 libdbus-1-3 libdrm2 libepoxy0 libgcrypt20 \
    libpciaccess0 libpixman-1-0 libselinux1 libsystemd0 libudev1 libunwind8 libxau6 \
    libxdmcp6 libxfont2 libxshmfence1 libx11-6 libx11-xcb1 libxcb-dri2-0 libxcb1 \
    libxcursor1 libxinerama1 libxrandr2 libxi6 libasound2 libpulse0 libxkbcommon0 libfontconfig1
  mkdir -p "$SIM_GPU_DIR/downloads" "$SIM_GPU_DIR/empty-config" "$SIM_GPU_DIR/rknn/include" "$SIM_GPU_DIR/rknn/lib"
  local cache="$SIM_GPU_DIR/downloads"
  local opi=https://raw.githubusercontent.com/orangepi-xunlong/rk-rootfs-build/adaf2dc40407566abee889deec7b6a8b5aaab133
  local rknn=https://raw.githubusercontent.com/airockchip/rknn-toolkit2/42aa1d426c0a9e0869b6374edba009f7208a1926/rknpu2/runtime/Linux/librknn_api
  simulator_download https://github.com/godotengine/godot-builds/releases/download/4.7.2-stable/Godot_v4.7.2-stable_linux.arm64.zip \
    5dd0d86405cf7e8adf79fb6377b38ba682a2846cb378ffe5364f38c01ad29b9d "$cache/godot.zip"
  simulator_download https://github.com/radxa-pkg/rockchip-xserver/releases/download/rk3528_linux_release-linux-5.10-gen-rkr6/xserver-xorg-core_1.20.11-1_arm64.deb \
    a13593a292a63b6e976ab3b22d60804309acadeeb41a4909b59ef9f773f9dcff "$cache/xorg.deb"
  simulator_download "$opi/common/libmali_linux6.1/libmali-valhall-g610-g13p0-x11-gbm_1.9-1_arm64.deb" \
    b9410026638cb4c8f0a203b31a70cdd48dc9c3c535666a5f45afef3837e79170 "$cache/mali.deb"
  simulator_download "$opi/common/rga2/librga2_2.2.0-1_arm64.deb" \
    5178aba61c6a57c09d592d158c58fbdaf841efc9bcf3054ca916f6d6a6886754 "$cache/rga.deb"
  simulator_download "$rknn/include/rknn_api.h" \
    c48e11a6f41b451a5fd1e4ad774ea60252d3d94f78bee9b21ea3d21b21deba9a "$SIM_GPU_DIR/rknn/include/rknn_api.h"
  simulator_download "$rknn/aarch64/librknnrt.so" \
    d31fc19c85b85f6091b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8 "$SIM_GPU_DIR/rknn/lib/librknnrt.so"
  unzip -o "$cache/godot.zip" -d "$SIM_GPU_DIR"
  chmod +x "$SIM_GPU_DIR/Godot_v4.7.2-stable_linux.arm64"
  dpkg-deb -x "$cache/xorg.deb" "$SIM_GPU_DIR/rockchip-xserver"
  dpkg-deb -x "$cache/mali.deb" "$SIM_GPU_DIR/mali"
  dpkg-deb -x "$cache/rga.deb" "$SIM_GPU_DIR/rga"
  # Do not replace GPU firmware already supplied by the board image.
  if [ ! -f /lib/firmware/mali_csffw.bin ]; then
    sudo install -m 644 "$SIM_GPU_DIR/mali/lib/firmware/mali_csffw.bin" /lib/firmware/mali_csffw.bin
    echo "GPU firmware installed. Reboot the Orange Pi, then rerun setup."
    exit 1
  fi
  cat > "$SIM_GPU_DIR/xorg-test.conf" <<'XORG'
Section "ServerFlags"
    Option "AutoAddDevices" "false"
    Option "AllowEmptyInput" "true"
EndSection
Section "Device"
    Identifier "Rockchip"
    Driver "modesetting"
    Option "AccelMethod" "glamor"
    Option "DRI" "2"
EndSection
Section "Monitor"
    Identifier "Monitor0"
    Option "PreferredMode" "1280x720"
EndSection
Section "Screen"
    Identifier "Screen0"
    Device "Rockchip"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1280x720"
    EndSubSection
EndSection
XORG
  touch "$SIM_GPU_DIR/xauth"
  chmod 600 "$SIM_GPU_DIR/xauth"
  xauth -f "$SIM_GPU_DIR/xauth" add :88 MIT-MAGIC-COOKIE-1 "$(python3 -c 'import secrets; print(secrets.token_hex(16))')"
  getent group render >/dev/null || sudo groupadd --system render
  getent group video >/dev/null || sudo groupadd --system video
  printf '%s\n' 'KERNEL=="mali[0-9]*", GROUP="render", MODE="0660"' \
    'SUBSYSTEM=="drm", KERNEL=="renderD*", GROUP="render", MODE="0660"' | sudo tee /etc/udev/rules.d/70-sunray-sim3d.rules >/dev/null
  sudo udevadm control --reload-rules
  sudo udevadm trigger --action=change --sysname-match='mali*'
  sudo udevadm trigger --action=change --subsystem-match=drm
  sudo udevadm settle
  export RKNN_INCLUDE_DIR="$SIM_GPU_DIR/rknn/include" RKNN_LIBRARY="$SIM_GPU_DIR/rknn/lib/librknnrt.so"
  setup_direct_link sunray-sim3d
  simulator_build
  simulator_install
  timeout 180 "$SIM_GPU_DIR/Godot_v4.7.2-stable_linux.arm64" --headless --editor --import --path "$SIM_ROOT/tools/sim3d"
  sudo systemctl start sunray-sim3d-world
  # A fresh JPEG proves that the real display renderer is working, not headless mode.
  python3 - <<'PY'
import pathlib, struct, time
frame = pathlib.Path('/run/sunray-sim3d/camera.frame')
for _ in range(120):
    try:
        b = frame.read_bytes()
        if len(b) > 100 and b[:4] == b'SRF1' and b[20:22] == b'\xff\xd8' and abs(time.time() - struct.unpack_from('<Q', b, 12)[0]/1000) < 2:
            break
    except FileNotFoundError:
        pass
    time.sleep(.5)
else:
    raise SystemExit('ERROR: no fresh rendered camera frame. See journalctl -u sunray-sim3d-world -u sunray-sim3d-display')
PY
  local invocation
  invocation=$(systemctl show sunray-sim3d-world -p InvocationID --value)
  if ! journalctl "_SYSTEMD_INVOCATION_ID=$invocation" --no-pager | grep -q 'Mali-G610'; then
    echo "ERROR: camera renders, but Mali-G610 hardware rendering was not confirmed."
    sudo systemctl stop "${SIM_UNITS[@]}"
    exit 1
  fi
  # Check actual NPU inference with the same user/groups as the Sunray service.
  local probe_dir
  probe_dir=$(mktemp -d)
  cat > "$probe_dir/probe.cpp" <<'NPU'
#include <rknn_api.h>
#include <vector>
#include <cstdio>
int main(int argc, char** argv) {
  if (argc != 2) return 1;
  rknn_context ctx = 0;
  if (rknn_init(&ctx, argv[1], 0, 0, nullptr) < 0) return 1;
  std::vector<unsigned char> pixels(640*360*3, 80);
  rknn_input input = {};
  input.buf = pixels.data(); input.size = pixels.size();
  input.type = RKNN_TENSOR_UINT8; input.fmt = RKNN_TENSOR_NHWC;
  bool ok = rknn_set_core_mask(ctx, RKNN_NPU_CORE_0) >= 0 &&
    rknn_inputs_set(ctx, 1, &input) >= 0 && rknn_run(ctx, nullptr) >= 0;
  rknn_destroy(ctx);
  if (ok) puts("NPU grass inference: OK");
  return ok ? 0 : 1;
}
NPU
  c++ -I "$RKNN_INCLUDE_DIR" "$probe_dir/probe.cpp" "$RKNN_LIBRARY" \
    -Wl,-rpath,"$SIM_GPU_DIR/rknn/lib" -o "$probe_dir/probe"
  if ! sudo systemd-run --quiet --wait --pipe --collect --property="User=$(id -un)" \
    --property='SupplementaryGroups=render video' --property=RuntimeMaxSec=30 \
    "$probe_dir/probe" "$SCRIPT_DIR/build-sim3d/models/grass.rknn"; then
    rm -rf "$probe_dir"
    echo "ERROR: NPU inference failed. Check the board image's rknpu kernel driver and journal."
    sudo systemctl stop "${SIM_UNITS[@]}"
    exit 1
  fi
  rm -rf "$probe_dir"
  sudo systemctl start sunray-sim3d
  sleep 5
  systemctl is-active --quiet sunray-sim3d
  sudo systemctl enable "${SIM_UNITS[@]}"
  echo "Simulator setup complete: running and enabled at boot."
  echo "The HDMI dummy must remain connected. A world/Pi restart resets the virtual position."
)

simulator_action() {
  case "${1:-help}" in
    setup) simulator_setup ;;
    build) simulator_build ;;
    install) simulator_install ;;
    start|restart)
      if systemctl is-active --quiet sunray; then
        echo "ERROR: the regular sunray service is running. Stop it before starting the simulator."
        return 1
      fi
      sudo systemctl "$1" sunray-sim3d ;;
    stop) sudo systemctl stop "${SIM_UNITS[@]}" ;;
    status) systemctl --no-pager status "${SIM_UNITS[@]}" ;;
    log) journalctl -u sunray-sim3d -u sunray-sim3d-world -n 100 -f ;;
    enable|disable) sudo systemctl "$1" "${SIM_UNITS[@]}" ;;
    help) echo "Usage: bash linux/service.sh simulator {setup|build|start|stop|restart|status|log}"
          echo "restart: Sunray only, preserving the running world. stop: all three services."
          echo "enable/disable: boot autostart only; does not start/stop services." ;;
    *) echo "Unknown simulator action: $1"; return 2 ;;
  esac
}

setup_direct_link() (
  set -eu
  local main_unit="${1:-sunray}"
  case "$main_unit" in sunray|sunray-sim3d) ;; *) echo "Unknown Sunray service"; exit 1;; esac
  if [ "$(uname -s)" != Linux ]; then echo "Direct-link setup requires Linux."; exit 1; fi
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-upgrade python3-venv
  local directory="$SIM_ROOT/tools/direct_link"
  python3 -m venv "$directory/.venv"
  "$directory/.venv/bin/python" -m pip install --only-binary=:all: -r "$directory/requirements.txt"
  local staging
  staging=$(mktemp -d)
  trap 'rm -rf "$staging"' EXIT
  case "$SIM_ROOT" in *[!a-zA-Z0-9_./-]*) echo "Unsupported checkout path"; exit 1;; esac
  cat > "$staging/sunray-direct.service" <<UNIT
[Unit]
Description=Sunray encrypted direct camera and joystick connection
After=network.target
[Service]
User=$(id -un)
WorkingDirectory=$directory
ExecStart=$directory/.venv/bin/python $directory/server.py
Restart=on-failure
RestartSec=3
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=read-only
PrivateTmp=true
UNIT
  sudo install -m 644 "$staging/sunray-direct.service" /etc/systemd/system/
  sudo mkdir -p "/etc/systemd/system/$main_unit.service.d"
  printf '[Unit]\nWants=sunray-direct.service\nAfter=sunray-direct.service\n' > "$staging/direct.conf"
  sudo install -m 644 "$staging/direct.conf" "/etc/systemd/system/$main_unit.service.d/direct.conf"
  sudo systemctl daemon-reload
  sudo systemctl restart sunray-direct.service
  echo "Direct-link helper installed. The Sunray build must enable DIRECT_LINK_ENABLE."
)

if [ "${1:-}" = direct-setup ]; then
  if [ "$EUID" -eq 0 ]; then echo "Please run as non-root (not sudo)"; exit 1; fi
  setup_direct_link "${2:-sunray}"
  exit $?
fi

# Noninteractive entry point, with no unrelated package installation or menu.
if [ "${1:-}" = simulator ]; then
  if [ "$EUID" -eq 0 ]; then echo "Please run as non-root (not sudo)"; exit 1; fi
  simulator_action "${2:-help}"
  exit $?
fi


echo "EUID=$EUID"
echo "PWD=$PWD"

KERNEL="$(uname -sr)"
CPU="$(uname -m)"
DISTRI="$(source /etc/os-release && echo "$ID")"

echo "CPU=$CPU  DISTRI=$DISTRI  KERNEL=$KERNEL"
 
cpu_ok=false
case "$CPU" in
  x86_64|amd64)          cpu_ok=true ;;   # Intel/AMD 64-bit
  aarch64|arm64)         cpu_ok=true ;;   # ARM 64-bit
  armv7l)                cpu_ok=true ;;   # ARM 32-bit
esac
if ! $cpu_ok; then
  echo "WARNING: probably unsupported CPU: $CPU (expected: x86_64/amd64, aarch64/arm64)"
fi

distri_ok=false
case "$DISTRI" in
  ubuntu)            distri_ok=true ;;   
  debian)            distri_ok=true ;;   
  raspbian)          distri_ok=true ;;   
esac
if ! $distri_ok; then
  echo "WARNING: probably unsupported distribution: $DISTRI (expected: ubuntu, debian, raspbian)"
fi

echo


CMD=""


if [ "$EUID" -eq 0 ]
  then echo "Please run as non-root (not sudo)"
  exit
fi


function build_sunray() {
  PACKAGES="cmake libbluetooth-dev libssl-dev libjpeg-dev"
  if ! dpkg -s $PACKAGES; then 
    echo "installing required Linux libraries... (you may have to enter your sudo password)"  
    sudo apt install -y $PACKAGES
  fi

  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi

  PS3='Please choose config: '
  list=$(ls config*.h)
  IFS=$'\n'
  select CONFIG_FILE in $list 
    do test -n "$CONFIG_FILE" && break; 
  exit; 
  done
  echo "selected: $CONFIG_FILE"

  CONFIG_PATHNAME=$PWD/$CONFIG_FILE
  sudo rm ../sunray/config.h
  rm -f CMakeCache.txt
  rm -f cmake_install.cmake
  rm -Rf CMakeFiles
  
  # Ensure build directory exists and cleanup safely inside it
  mkdir -p build
  cd build || { echo "ERROR: failed to enter build directory"; return 1; }
  if [[ "$(basename "$PWD")" != "build" ]]; then
    echo "ERROR: safety check failed (not in build dir), aborting cleanup"
    return 1
  fi
  # Remove previous build artifacts only inside build directory (including dotfiles)
  shopt -s dotglob nullglob
  rm -rf -- *
  shopt -u dotglob nullglob
  
  #exit
  cmake -D CONFIG_FILE=$CONFIG_PATHNAME ..
  make 
  cd ..
}


function rebuild_sunray() {
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi
  cd build
  make
  cd ..
}


function install_sunray_alfred() {
  #if [ "$EUID" -eq 0 ]
  #  then echo "Please run as non-root (not sudo)"
  #  exit
  #fi
  echo "NOTE: enter user password if being asked (see Sunray PDF manual for password)"
  echo "stopping sunray service..."
  sudo systemctl stop sunray
  echo "installing executable..."
  cp build/sunray ~/sunray_install/
  echo "starting sunray service..."
  sudo systemctl start sunray
}

# start RecoverCAN service 
function start_recovercan_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  if [[ `pidof recovercan` != "" ]]; then
        echo "recovercan service already running! Exiting..."
        exit
  fi
  echo "starting recovercan service..."
  #cp config_files/motion/motion.conf /etc/motion

  REPLACEPATH="/home/pi/Sunray/linux"
  sed "s+$REPLACEPATH+$PWD+g" <$PWD/config_files/recovercan/recovercan.service.example >$PWD/config_files/recovercan/recovercan.service

  sudo cp $PWD/config_files/recovercan/recovercan.service /etc/systemd/system/recovercan.service
  sudo chmod 644 /etc/systemd/system/recovercan.service
  sudo systemctl daemon-reload
  sudo systemctl enable recovercan
  sudo systemctl start recovercan
  sudo systemctl --no-pager status recovercan
  echo "recovercan service started! (see log with: 'journalctl -f -u recovercan')"
}

function stop_recovercan_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping recovercan service..."
  sudo systemctl stop recovercan
  sudo systemctl disable recovercan
  echo "recovercan service stopped!"
}


# start USB camera streaming web server 
function start_cam_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  if [[ `pidof motion` != "" ]]; then
        echo "motion app already running! Exiting..."
        exit
  fi
  if [ ! -d /etc/motion ]; then
    sudo apt-get -y install motion || return 1
  fi
  echo "starting motion service..."
  cp config_files/motion/motion.conf /etc/motion
  sudo cp $PWD/config_files/motion/motion.service /etc/systemd/system/motion.service
  sudo chmod 644 /etc/systemd/system/motion.service
  sudo systemctl daemon-reload
  sudo systemctl enable motion
  sudo systemctl start motion
  sudo systemctl --no-pager status motion
  echo "motion service started!"
}

function stop_cam_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping motion service..."
  sudo systemctl stop motion
  sudo systemctl disable motion
  echo "motion service stopped!"
}

function start_sunray_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi  
  if [[ `pidof sunray` != "" ]]; then
        echo "Sunray linux app already running! Exiting..."
        exit
  fi
  # enable sunray service
  echo "starting sunray service..."
  #ln -s /home/pi/sunray_install/config_files/sunray.service /etc/systemd/system/sunray.service
  REPLACEPATH="/home/pi/Sunray/linux"
  sed "s+$REPLACEPATH+$PWD+g" <$PWD/config_files/sunray/sunray.service.example >$PWD/config_files/sunray/sunray.service

  sudo cp $PWD/config_files/sunray/sunray.service /etc/systemd/system/sunray.service
  sudo chmod 644 /etc/systemd/system/sunray.service
  sudo mkdir -p /boot/sunray
  sudo chmod 644 /boot/sunray
  sudo systemctl daemon-reload
  sudo systemctl enable sunray
  sudo systemctl start sunray
  sudo systemctl --no-pager status sunray
  echo "sunray service started!"
}

function stop_sunray_service() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  # disable sunray service
  echo "stopping sunray service..."
  sudo systemctl stop sunray
  sudo systemctl disable sunray
  echo "sunray service stopped!"
}


function start_sunray() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"
  sudo ./start_sunray.sh
}

function start_sunray_motor_test() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"
  sudo ./start_sunray.sh <<< "AT+E"
}

function start_sunray_sensor_test() {
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting sunray... (press CTRL+C to stop)"  
  sudo ./start_sunray.sh <<< "AT+F"
}



function start_log_service(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi  
  echo "starting logging service..."
  sudo service rsyslog start
}

function stop_log_service(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping logging service..."
  sudo service rsyslog stop
}

function start_dm(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "starting display manager..."
  sudo service lightdm start
}

function stop_dm(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "stopping display manager..."
  sudo service lightdm stop
}

function list_services(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "listing services..."
  sudo service --status-all
}

function show_log(){
  echo "show log... (press CTRL+C to stop)"
  journalctl -f -u sunray
}

function save_log(){
  echo "saving log.txt..."
  journalctl -u sunray > log.txt
}

function clear_log(){
  #if [ "$EUID" -ne 0 ]
  #  then echo "Please run as root (sudo)"
  #  exit
  #fi
  echo "clearing log..."
  sudo journalctl --rotate --vacuum-time=1s -u sunray
}

function kernel_log(){
  echo "kernel log... (press CTRL+C to stop)"
  dmesg -wH
}


function update_local_repository(){
  if ! command -v git &> /dev/null
  then
    sudo apt-get -y install git
  fi
  git pull
  echo "restarting script file..."
  exec "$0" "$@"
}


function i2c_scanner(){
  if ! command -v i2cdetect &> /dev/null
  then
    sudo apt-get -y install i2c-tools
  fi
  echo "Note: there should be detected at least one I2C device (e.g. IMU at address 0x69)" 
  sudo i2cdetect -y 1
}

function show_network_interfaces(){
  if ! command -v ifconfig &> /dev/null
  then
    sudo apt-get -y install net-tools
  fi
  sudo ip link set can0 up type can bitrate 1000000
  echo "------Note: there should be listed a CAN interface (e.g. can0)------"
  ifconfig -s
  echo "------------"
}


function gpio_status(){
  if ! command -v gpio &> /dev/null
  then
    sudo apt-get -y install wiringpi
  fi
  echo "------Note: for owlPCB, physical pin12 should be V=0 (mode down)------"
  gpio readall
}


function linux_info(){
  if ! command -v screenfetch &> /dev/null
  then
    sudo apt-get -y install screenfetch
  fi
  screenfetch
}


function upgrade_linux_system(){
  read -p "Upgrade all linux system packages? (y/n): " answer
  if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
    sudo apt-get update -y && sudo apt-get upgrade -y
  fi
}


function network_manager(){
  if ! command -v nmtui &> /dev/null
  then
    sudo apt-get -y install network-manager
  fi
  nmtui 
}


function fix_local_repository_file_permissions(){
  USER=$(whoami)
  echo "changing all files' owner to: $USER..."
  sudo chown -R $USER:$USER ../  
}


function install_anydesk(){
  if ! command -v wget &> /dev/null
  then
    sudo apt-get -y install wget
  fi
  CURDIR=$(pwd)
  cd /tmp

  if [[ "$CPU" == "aarch64" || "$CPU" == "arm64" ]]; then
    echo "Installing AnyDesk for ARM64 ..."
    FILE="anydesk_7.0.2-1_arm64.deb"
    URL="https://download.anydesk.com/rpi/$FILE"
  elif [[ "$CPU" == "armv7l" || "$CPU" == "armv6l" ]]; then
    echo "Installing AnyDesk for ARMHF ..."
    FILE="anydesk_6.3.0-1_armhf.deb"
    URL="https://download.anydesk.com/rpi/$FILE"
  else
    echo "Unknown CPU: $CPU"
    return
  fi

  wget -O "$FILE" "$URL"
  sudo apt install -y "./$FILE"

  echo "Starting AnyDesk service..."
  sudo systemctl daemon-reload
  sudo systemctl enable anydesk
  sudo systemctl restart anydesk
  for i in {1..10}; do
    sudo systemctl is-active --quiet anydesk && break
    sleep 1
  done
  if ! sudo systemctl is-active --quiet anydesk; then
    echo "ERROR: AnyDesk service is not running"
    sudo systemctl --no-pager status anydesk
    cd "$CURDIR"
    return 1
  fi

  echo
  read -s -p "Choose AnyDesk password: " PASS
  echo
  if ! printf '%s\n' "$PASS" | sudo anydesk --set-password; then
    echo "ERROR: failed to set AnyDesk password"
    sudo systemctl --no-pager status anydesk
    cd "$CURDIR"
    return 1
  fi
  
  echo
  echo "AnyDesk ID:"
  if ! sudo anydesk --get-id; then
    echo "ERROR: failed to read AnyDesk ID"
    sudo systemctl --no-pager status anydesk
    cd "$CURDIR"
    return 1
  fi
  echo
  echo "On your PC, connect to your PI with the displayed ID and choosen password"

  cd "$CURDIR"
}


function flash_ublox_receiver(){
  ../ublox_f9p_configs/flash.sh
}







# if [ ! -d "/home/pi/Sunray" ]; then
#  echo "install Sunray first!"
#  exit
# fi

#systemctl status sunray
#systemctl status motion


compile_menu () {
    echo "Compile and test menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Build sunray executable (you can choose config file)" 
        "Rebuild sunray executable (using last choosen config file)"
        "Run sunray executable"
        "Run sunray executable with motor test (WARNING: motors will start with full speed, jack-up your robot!)"
        "Run sunray executable with sensor test"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                build_sunray
                break
            ;;
            ${options[1]})
                rebuild_sunray
                break
            ;;
            ${options[2]})
                start_sunray
                break
            ;;
            ${options[3]})
                start_sunray_motor_test
                break
            ;;
            ${options[4]})
                start_sunray_sensor_test
                break
            ;;
            ${options[5]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}



linux_services_menu () {
    echo "Linux services menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Install sunray executable on existing Alfred file system (Alfred-only)" 
        "Start sunray service (as Linux autostart)"
        "Stop sunray service"
        "Start RecoverCAN service (as Linux autostart)"
        "Stop RecoverCAN service"
        "Start camera service (as Linux autostart)"
        "Stop camera service"
        "Start Linux logging service"
        "Stop Linux logging service"
        "Start display manager"
        "Stop display manager"
        "List services"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                install_sunray_alfred
                break
            ;;
            ${options[1]})
                start_sunray_service
                break
            ;;
            ${options[2]})
                stop_sunray_service
                break
            ;;
            ${options[3]})
                start_recovercan_service
                break
            ;;
            ${options[4]})
                stop_recovercan_service
                break
            ;;
            ${options[5]})
                start_cam_service
                break
            ;;
            ${options[6]})
                stop_cam_service
                break
            ;;
            ${options[7]})
                start_log_service
                break
            ;;
            ${options[8]})
                stop_log_service
                break
            ;;
            ${options[9]})
                start_dm
                break
            ;;
            ${options[10]})
                stop_dm
                break
            ;;
            ${options[11]})
                list_services
                break
            ;;
            ${options[12]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}



linux_logging_menu () {
    echo "Linux logging menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Show log" 
        "Save log"
        "Clear log"
        "Kernel log"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                show_log
                break
            ;;
            ${options[1]})
                save_log
                break
            ;;
            ${options[2]})
                clear_log
                break
            ;;
            ${options[3]})
                kernel_log
                break
            ;;
            ${options[4]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}


linux_diag_menu () {
    echo "Linux diagnostics menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(        
        "Linux info"
        "I2C scanner"
        "Show network/CAN interfaces"
        "GPIO status"
        "Network manager (WiFi etc.)"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                linux_info
                break
            ;;
            ${options[1]})
                i2c_scanner
                break
            ;;
            ${options[2]})
                show_network_interfaces
                break
            ;;
            ${options[3]})
                gpio_status
                break
             ;;
            ${options[4]})
                network_manager
                break
             ;;
            ${options[5]})              
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}




update_menu () {
    echo "Update menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Update local Sunray repository from remote Github repository" 
        "Fix local Sunray repository file permissions"
        "Upgrade all Linux system packages"
        "Install AnyDesk"
        "Flash ublox receiver firmware"
        "Back"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                update_local_repository
                break
            ;;
            ${options[1]})
                fix_local_repository_file_permissions
                break
            ;;
            ${options[2]})
                upgrade_linux_system
                break
            ;;
            ${options[3]})
                install_anydesk
                break
            ;;
            ${options[4]})
                flash_ublox_receiver
                break
            ;;
            ${options[5]})
                return
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
}


simulator_menu () {
    local option
    local PS3='Simulator action: '
    local options=(
        "Set up / update simulator (GPU, build, services and autostart)"
        "Start simulator"
        "Stop simulator"
        "Status / diagnostics"
        "Back"
    )
    select option in "${options[@]}"; do
        case "$REPLY" in
            1) simulator_action setup; break ;;
            2) simulator_action start; break ;;
            3) simulator_action stop; break ;;
            4) simulator_action status; journalctl -u sunray-sim3d -u sunray-sim3d-world -u sunray-sim3d-display -n 40 --no-pager; break ;;
            5) return ;;
            *) echo "Invalid option" ;;
        esac
    done
}


main_menu () {
    echo "Main menu (NOTE: press CTRL+C to stop any pending actions)"
    options=(
        "Compile and test menu"
        "Linux services menu"
        "Linux logging menu"
        "Linux diagnostics menu"
        "Update menu"
        "3D simulator menu (Orange Pi)"
        "Quit"
    )
    select option in "${options[@]}"; do
        case $option in
            ${options[0]})
                compile_menu
                return 0
            ;;
            ${options[1]})
                linux_services_menu
                return 0
            ;;
            ${options[2]})
                linux_logging_menu
                return 0
            ;;
            ${options[3]})
                linux_diag_menu
                return 0
            ;;
            ${options[4]})
                update_menu
                return 0
            ;;
            ${options[5]})
                simulator_menu
                return 0
             ;;
            ${options[6]})
                exit
             ;;
            *) 
                echo invalid option
            ;;
        esac
    done
    exit 0
}


# show main menu
while true
do 
  main_menu
done

