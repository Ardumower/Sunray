#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
${CXX:-c++} -std=c++14 -Wall -Wextra linux/tests/ublox-signal-diagnostics-test.cpp -o "$work/ublox-signals"
"$work/ublox-signals"
