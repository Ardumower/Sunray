#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
python3 - "$work" <<'PY'
from pathlib import Path
import sys
s=Path('sunray/src/camera/CameraStreamer.cpp').read_text()
a=s.index('void CameraStreamer::runLoop()');b=s.index('void CameraStreamer::runCaptureSession()',a)
Path(sys.argv[1],'retry.inc').write_text(s[a:b])
a=s.index('  struct CaptureResources {');b=s.index('  } capture(fd);',a)
Path(sys.argv[1],'resources.inc').write_text(s[a:b]+'  };\n')
# No recursive reopen and no unchecked setup failure may terminate the worker.
session=s[s.index('void CameraStreamer::runCaptureSession()'):s.index('extern "C" void cameraStreamerStart',s.index('void CameraStreamer::runCaptureSession()'))]
assert 'runLoop();' not in session
assert 'O_NONBLOCK' in session and 'errno==EAGAIN' in session
assert 'reflectorWorker.stop();' in session
assert 'CAM streamon failed' in session and 'CAM dequeue failed' in session
PY
"${CXX:-c++}" -std=c++11 -pthread -I"$work" linux/tests/camera-reconnect-test.cpp -o "$work/test"
"$work/test"
