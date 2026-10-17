#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -Isunray linux/tests/camera-gps-position-test.cpp -o "$work/gps"
"$work/gps"
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -Isunray linux/tests/camera-gps-heading-test.cpp -o "$work/heading"
"$work/heading"
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -Isunray linux/tests/reflector-docking-gate-test.cpp -o "$work/gate"
"$work/gate"
python3 - "$work" <<'PY'
from pathlib import Path
import sys
out=Path(sys.argv[1])
def method(file,signature):
 s=Path(file).read_text();a=s.index(signature);b=s.index('{',a);depth=1;e=b+1
 while depth:
  depth+=(s[e]=='{')-(s[e]=='}');e+=1
 return s[a:e]
out.joinpath('obstacle-production.inc').write_text('\n'.join(method(f,s) for f,s in [
 ('sunray/src/op/Op.cpp','void Op::changeOp('),
 ('sunray/src/op/Op.cpp','void Op::changeOperationTypeByOperator('),
 ('sunray/src/op/Op.cpp','bool Op::stopOnDockingObstacle('),
 ('sunray/src/op/Op.cpp','void Op::onChargerConnected('),
 ('sunray/src/op/ErrorOp.cpp','void ErrorOp::run('),
 ('sunray/src/op/ErrorOp.cpp','void ErrorOp::end(')]))
s=Path('sunray/LineTracker.cpp').read_text();a=s.index('    mow = false;',s.index('if (stateEstimator.stateLocalizationMode == LOC_REFLECTOR_STRIPS)'));b=s.index('    static unsigned long nextReflectorStatusTime',a)
out.joinpath('tracking-production.inc').write_text(s[a:b])
s=Path('sunray/src/op/MowOp.cpp').read_text()
a=s.index('#if defined(UNDOCK_TEST_TIMEOUT_MS)')
b=s.index('\n#ifdef DOCK_REFLECTOR_STRIPS\n    if (maps.isUndocking()',a)
out.joinpath('undock-test-production.inc').write_text(s[a:b])
PY
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/reflector-docking-safety-test.cpp -o "$work/safety"
"$work/safety"
