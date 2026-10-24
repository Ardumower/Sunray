#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
${CXX:-c++} -std=c++14 -Wall -Wextra linux/tests/dock-slip-test.cpp -o "$work/test"
"$work/test"
