#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
# Build an isolated source copy: never replace the user's robot config.h.
mkdir -p "$work/sunray/src/driver" "$work/linux/tests"
cp -R sunray/src/ublox sunray/src/net "$work/sunray/src/"
cp sunray/src/driver/RobotDriver.h "$work/sunray/src/driver/"
cp sunray/gps.h sunray/types.h sunray/events.h "$work/sunray/"
cp linux/tests/gps-cloud-isolation-test.cpp "$work/linux/tests/"
cat > "$work/sunray/config.h" <<'EOF'
#define CONSOLE Console
#define GPS_CONFIG false
#define GPS_CONFIG_FILTER false
#define GPS_CONFIG_DGNSS_TIMEOUT 60
#define CPG_CONFIG_FILTER_MINELEV 10
#define CPG_CONFIG_FILTER_NCNOTHRS 10
#define CPG_CONFIG_FILTER_CNOTHRS 30
EOF
cxx=${CXX:-g++}
"$cxx" -std=c++14 -pthread -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -I linux/src "$work/linux/tests/gps-cloud-isolation-test.cpp" \
  "$work/sunray/src/ublox/ublox.cpp" \
  "$work/sunray/src/ublox/SparkFun_Ublox_Arduino_Library.cpp" \
  linux/src/Stream.cpp linux/src/WString.cpp linux/src/Print.cpp \
  linux/src/Console.cpp linux/src/IPAddress.cpp linux/src/Wire.cpp \
  -o "$work/gps-cloud"
"$work/gps-cloud"
