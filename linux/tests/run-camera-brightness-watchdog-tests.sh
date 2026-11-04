#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Isunray linux/tests/camera-brightness-watchdog-test.cpp -o "$work/test"
"$work/test"
