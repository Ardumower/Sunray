#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -D__linux__ -std=c++14 -O2 -Wall -Wextra ${CXXFLAGS:-} -pthread -Isunray linux/tests/reflector-failure-recorder-test.cpp -o "$work/test"
"$work/test"
