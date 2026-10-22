#!/bin/sh
# Optional arguments: illumination mode (0..11), variant (0..7), strip count (0..4).
set -eu
cd "$(dirname "$0")/../.."
REFLECTOR_TEST_SOURCE=linux/tests/reflector-lighting-test.cpp \
  sh linux/tests/run-reflector-strips-tests.sh "$@"
