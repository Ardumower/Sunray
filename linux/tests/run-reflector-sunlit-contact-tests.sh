#!/bin/sh
# Real charging-contact captures: sunlit backing, glare and auto-exposure changes.
set -eu
cd "$(dirname "$0")/../.."
# Sunrise recordings from mscmower on 2026-10-26: glare, auto-exposure,
# side clipping and independent horizontal shadows. Native 035/037 retain
# the full-resolution JPEG; resizing changes the measured contours.
# Repeat each frame to verify acquisition, then continuity across captures.
for fixture in "$PWD"/linux/tests/fixtures/reflector-sunlit-contact*.jpg; do
  printf '%s\n' "$fixture" "$fixture" "$fixture"
done | sh linux/tests/run-reflector-sequence-tests.sh
