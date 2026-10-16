#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
"${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined jpeg-decode-test.cpp -ljpeg -o "$build_dir/test"
ASAN_OPTIONS=detect_leaks=1 "$build_dir/test"
