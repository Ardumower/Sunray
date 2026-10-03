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
cp linux/tests/gps-startup-test.cpp "$work/linux/tests/"
cp linux/config_xlmower.h "$work/sunray/config.h"
cp linux/tests/gps-motor-fakes.h "$work/sunray/robot.h"
cp sunray/motor.cpp sunray/motor.h sunray/pid.cpp sunray/pid.h \
  sunray/lowpass_filter.cpp sunray/lowpass_filter.h sunray/helper.h "$work/sunray/"
cxx=${CXX:-g++}
"$cxx" -std=c++14 -pthread -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -I linux/src -I sunray "$work/linux/tests/gps-startup-test.cpp" \
  "$work/sunray/src/ublox/ublox.cpp" \
  "$work/sunray/motor.cpp" "$work/sunray/pid.cpp" "$work/sunray/lowpass_filter.cpp" \
  linux/src/Stream.cpp linux/src/WString.cpp linux/src/Print.cpp \
  linux/src/Console.cpp linux/src/IPAddress.cpp linux/src/Wire.cpp \
  -o "$work/gps-startup"
"$work/gps-startup"
