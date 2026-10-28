#!/bin/sh
# Build the Cloud simulator with real NPU inference; never silently omit grass.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
if [ -z "${RKNN_INCLUDE_DIR:-}" ] && [ -f "$HOME/rknn_model_zoo/3rdparty/rknpu2/include/rknn_api.h" ]; then
  RKNN_INCLUDE_DIR="$HOME/rknn_model_zoo/3rdparty/rknpu2/include"
fi
set -- -S "$root/linux" -B "$root/linux/build-sim3d" \
  -D "CONFIG_FILE=$root/linux/config_sim_ws.h" -D SUNRAY_REQUIRE_RKNN=ON
if [ -n "${RKNN_INCLUDE_DIR:-}" ]; then set -- "$@" -D "RKNN_INCLUDE_DIR=$RKNN_INCLUDE_DIR"; fi
if [ -n "${RKNN_LIBRARY:-}" ]; then set -- "$@" -D "RKNN_LIBRARY=$RKNN_LIBRARY"; fi
cmake "$@"
cmake --build "$root/linux/build-sim3d" -j "${BUILD_JOBS:-4}"
