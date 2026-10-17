#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -pthread -Wall -Wextra -Isunray linux/tests/camera-light-logger-test.cpp -o "$work/test"
output=$("$work/test")
python3 - "$output" <<'PY'
from pathlib import Path
import json,sys,shutil
root=Path(sys.argv[1]);rows=[json.loads(s) for p in (root/'camera_light').glob('*.jsonl') for s in p.read_text().splitlines()]
assert len(rows)==2,rows
first,last=rows
assert first['sequence_gaps']==2 and first['invalid_frames']==1
assert first['buffer_timestamp_fps']==10 and first['select_timeouts']==1
assert first['detection_count']==2 and first['found_count']==1
assert first['regions_2x3']==[0,40,80,120,160,200]
assert first['exposure_raw_100us'] is None and first['gain_raw'] is None
assert first['reference_jpeg'] is None
assert last['image_valid'] is False and 'luma_mean' not in last
assert last['capture_fps']==0 and last['select_timeouts']==1
assert last['sequence_gaps']==0 and last['detection_count']==0
shutil.rmtree(root)
print('PASS light statistics, capture timestamps/gaps, per-window resets, unavailable controls and image outages')
PY
