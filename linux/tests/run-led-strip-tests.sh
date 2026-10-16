#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/src/camera"
cp "$root/sunray/src/camera/LedStripDetector.cpp" "$root/sunray/src/camera/LedStripDetector.h" "$work/src/camera/"
touch "$work/config.h"
"${CXX:-c++}" -D__linux__ -std=c++11 -O2 -Wall -Wextra ${CXXFLAGS:-} -I"$work" "$root/linux/tests/led-strip-test.cpp" "$work/src/camera/LedStripDetector.cpp" -o "$work/test"
"$work/test" "$@"
