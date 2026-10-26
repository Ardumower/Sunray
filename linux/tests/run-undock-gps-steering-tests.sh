#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
python3 - "$work" <<'PY'
from pathlib import Path
import sys
s=Path('sunray/LineTracker.cpp').read_text();out=Path(sys.argv[1])
a=s.index('    angular =  p * trackerDiffDelta');b=s.index('    /*pidLine.w',a)
out.joinpath('undock-steering-production.inc').write_text(s[a:b])
a=s.index('      linear = DOCK_LINEAR_SPEED;');b=s.index('\n    } else if',a)
out.joinpath('undock-speed-production.inc').write_text(s[a:b])
PY
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/undock-gps-steering-test.cpp -o "$work/test"
"$work/test"
