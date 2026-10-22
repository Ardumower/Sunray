#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
mkdir -p "$work/src/camera"
cp sunray/src/camera/ReflectorStripsDetector.cpp sunray/src/camera/ReflectorStripsDetector.h "$work/src/camera/"
touch "$work/config.h"
"${CXX:-c++}" -D__linux__ -std=c++11 -O2 -Wall -Wextra ${CXXFLAGS:-} -I"$work" "${REFLECTOR_TEST_SOURCE:-linux/tests/reflector-strips-test.cpp}" "$work/src/camera/ReflectorStripsDetector.cpp" -o "$work/test"
"$work/test" "$@"
