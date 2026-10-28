#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
for name in camera-avoidance-test camera-avoidance-motion-test camera-plan-worker-test; do
  "${CXX:-c++}" -std=c++14 -O2 -pthread -I linux/src -I sunray "linux/tests/$name.cpp" -o "$work/$name"
  "$work/$name"
done
if [ "$(uname -s)" = Linux ]; then
  printf '#pragma once\n' > "$work/config.h"
  "${CXX:-c++}" -std=c++14 -O2 -pthread -DSUNRAY_RKNN=1 -I "$work" -I linux/src -I sunray \
    linux/tests/camera-avoidance-runtime-test.cpp linux/src/CameraAvoidanceRuntime.cpp -o "$work/runtime"
  "$work/runtime"
fi
