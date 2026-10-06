#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cxx=${CXX:-g++}
# Compile the actual Arduino timing implementation, without hardware startup/main.
# Linker wrappers change clocks only inside this test process, never the Pi clock.
"$cxx" -std=c++14 -pthread -DNO_MAIN -ffunction-sections -fdata-sections \
  -I linux/src linux/src/wiring_main.cpp linux/tests/monotonic-time-test.cpp \
  -Wl,--gc-sections -Wl,--wrap=clock_gettime -Wl,--wrap=gettimeofday \
  -o "$work/monotonic-time"
"$work/monotonic-time"
"$work/monotonic-time" --real
