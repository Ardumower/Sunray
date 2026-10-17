#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
# Exercise the actual method without linking robot hardware and navigation globals.
python3 - "$work/docking-heading-production.inc" <<'PY'
import pathlib,sys
source=pathlib.Path('sunray/map.cpp').read_text()
start=source.index('bool Map::getDockingPos(')
brace=source.index('{',start)
depth=1
end=brace+1
while depth:
    depth+=(source[end]=='{')-(source[end]=='}')
    end+=1
pathlib.Path(sys.argv[1]).write_text(source[start:end])
PY
"${CXX:-c++}" -std=c++11 -Wall -Wextra -I"$work" linux/tests/docking-heading-test.cpp -o "$work/test"
"$work/test"
