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
assert 'SunsetDock::shouldDock(sunsetDockingRequired(),activeOp->getGoalOperationType()==OP_MOW)' in robot
start=robot[robot.index('void setOperation(OperationType op, bool allowRepeat)'):]
assert start.index('op == OP_MOW && sunsetDockingRequired()') < start.index('stateEstimator.stateOp = op')
assert 'mowing start rejected during night pause' in start
assert 'maps.dockPoints.numPoints>=2' in robot
assert 'STA_UNSYNC' in robot
charge=Path('sunray/src/op/ChargeOp.cpp').read_text()
assert '!sunsetDockingRequired() && timetable.shouldAutostartNow()' in charge
assert charge.index('mowOp.setInitiatedByOperator(false)') < charge.index('changeOp(mowOp)')
comm=Path('sunray/comm.cpp').read_text()
manual=comm[comm.index('void Comm::cmdMotor('):comm.index('void Comm::cmdMotorTest(')]
assert 'sunsetDockingRequired' not in manual
assert 'setInitiatedByOperator' not in manual
assert '#define SUNSET_DOCK_ENABLE false' in Path('sunray/config_example.h').read_text()
assert '#define SUNSET_DOCK_ENABLE true' in Path('linux/config_xlmower.h').read_text()
print('PASS scheduler integration: mow-only trigger, dock-path guard, synchronized time, charge restart gate, opt-in default')
PY
