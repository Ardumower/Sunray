#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
mkdir -p "$work/src/camera"
cp sunray/src/camera/ReflectorStripsDetector.cpp sunray/src/camera/ReflectorStripsDetector.h sunray/src/camera/SafeJpeg.h "$work/src/camera/"
touch "$work/config.h"
"${CXX:-c++}" -D__linux__ -std=c++11 -O2 -pthread ${CXXFLAGS:-} -I"$work" linux/tests/reflector-sequence-test.cpp "$work/src/camera/ReflectorStripsDetector.cpp" ${LDFLAGS:-} -ljpeg -o "$work/test"
"$work/test"
