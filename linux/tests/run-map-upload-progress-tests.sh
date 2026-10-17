#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
python3 - "$work/map-upload-production.inc" <<'PY'
from pathlib import Path
import sys
s=Path('sunray/map.cpp').read_text();methods=[]
for signature in ['void Map::finishedUploadingMap(', 'void Map::clearMap(', 'bool Map::setPoint(', 'bool Map::setWayCount(', 'bool Map::setExclusionLength(']:
 a=s.index(signature);b=s.index('{',a);e=b+1;depth=1
 while depth:
  depth+=(s[e]=='{')-(s[e]=='}');e+=1
 methods.append(s[a:e])
Path(sys.argv[1]).write_text('\n'.join(methods))
PY
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/map-upload-progress-test.cpp -o "$work/test"
"$work/test"
