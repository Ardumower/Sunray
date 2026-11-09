#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Isunray linux/tests/sunset-dock-test.cpp -o "$work/test"
"$work/test"
python3 - <<'PY'
from pathlib import Path
robot=Path('sunray/robot.cpp').read_text()
assert 'SunsetDock::shouldDock(sunsetDockingRequired(),activeOp->getGoalOperationType()==OP_MOW,mowOp.initiatedByOperator)' in robot
assert 'if(op==OP_MOW)mowOp.setInitiatedByOperator(true)' in robot
assert 'maps.dockPoints.numPoints>=2' in robot
assert 'STA_UNSYNC' in robot
charge=Path('sunray/src/op/ChargeOp.cpp').read_text()
assert '!sunsetDockingRequired() && timetable.shouldAutostartNow()' in charge
assert charge.index('mowOp.setInitiatedByOperator(false)') < charge.index('changeOp(mowOp)')
comm=Path('sunray/comm.cpp').read_text()
assert '(linear!=0 || angular!=0) && activeOp->getGoalOperationType()==OP_MOW' in comm
assert '#define SUNSET_DOCK_ENABLE false' in Path('sunray/config_example.h').read_text()
assert '#define SUNSET_DOCK_ENABLE true' in Path('linux/config_xlmower.h').read_text()
print('PASS scheduler integration: mow-only trigger, dock-path guard, synchronized time, charge restart gate, opt-in default')
PY
