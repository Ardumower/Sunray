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
s=Path('sunray/src/op/DockOp.cpp').read_text()
a=s.index('  bool automaticCameraEntry = false;')
b=s.index('  if (confirmedCameraEntry || automaticCameraEntry)',a)
out.joinpath('dock-entry-production.inc').write_text(s[a:b])
s=Path('sunray/src/op/MowOp.cpp').read_text()
a=s.index('    float dockX,dockY,dockHeading;')
b=s.index('    if (resumeCamera)',a)
out.joinpath('undock-entry-production.inc').write_text(s[a:b])
s=Path('sunray/robot.cpp').read_text()
a=s.index('  // Pure rotation need not change GPS position.')
b=s.index('  return false;',a)
out.joinpath('gps-motion-production.inc').write_text(s[a:b])
out.joinpath('dock-search-production.inc').write_text(method('sunray/src/op/DockOp.cpp','bool DockOp::runCameraSearch('))
out.joinpath('dock-contact-production.inc').write_text('\n'.join(method('sunray/src/op/DockOp.cpp', sig) for sig in ['void DockOp::run(', 'void DockOp::onChargerConnected(']))
s=Path('sunray/StateEstimator.cpp').read_text()
a=s.index('      const bool contactAdvance =');b=s.index('\n',s.index('motor.stopImmediately(false);',a))
out.joinpath('dock-contact-estimator.inc').write_text(s[a:b])

# Running RTK approach must use StateEstimator's route-aware handoff gate.
assert 'tryCameraEntry(' not in method('sunray/src/op/DockOp.cpp','void DockOp::run(')
assert 'tryCameraEntry(' in method('sunray/src/op/DockOp.cpp','void DockOp::begin(')
s=Path('sunray/src/op/DockOp.cpp').read_text()
a=s.index('    if (bumper.obstacle()')
b=s.index('#ifdef DOCK_REFLECTOR_STRIPS',a)
out.joinpath('dock-search-prefix.inc').write_text(s[a:b])
s=Path('sunray/motor.cpp').read_text()
a=s.index('#ifdef DOCK_MAX_WHEEL_SPEED')
b=s.index('   float rspeed',a)
out.joinpath('dock-speed-production.inc').write_text(s[a:b])
PY
"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/reflector-docking-safety-test.cpp -o "$work/safety"
"$work/safety"

"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/camera-entry-test.cpp -o "$work/entry"
"$work/entry"

"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" linux/tests/gps-rotation-motion-test.cpp -o "$work/rotation"
"$work/rotation"

"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/dock-marker-search-test.cpp -o "$work/search"
"$work/search"

"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/dock-speed-policy-test.cpp -o "$work/speed"
"$work/speed"

"${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" -Isunray linux/tests/dock-contact-advance-test.cpp -o "$work/contact"
"$work/contact"
