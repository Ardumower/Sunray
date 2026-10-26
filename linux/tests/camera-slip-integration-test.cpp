#include "src/navigation/ReflectorDockingGate.h"
#include "src/navigation/DockSlipRecovery.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
using std::min;using std::max;using std::abs;
enum {OP_IDLE,OP_DOCK,OP_MOW,OP_ERROR,SENS_BUMPER,SENS_OBSTACLE};
struct {int stateOp=OP_DOCK,stateSensor=0;float stateDelta=0;ReflectorDockingGate reflectorDockingGate;} stateEstimator;
struct {bool undock=false;bool isUndocking(){return undock;}} maps;
struct {float ticksPerCm=10;unsigned long motorLeftTicks=10000,motorRightTicks=10000;int stops=0;void stopImmediately(bool){++stops;}} motor;
struct {bool hit=false;bool obstacle(){return hit;}} bumper;
struct {template<class T>void print(T){} template<class T>void print(T,int){} template<class T>void println(T){} } CONSOLE;
struct Op {int changes=0;void changeOp(Op&){++changes;}} op,errorOp;Op*activeOp=&op;
struct {bool isPlaying(){return true;}void sound(int,bool){}} buzzer;
unsigned long now=0;
unsigned long millis(){return now;}
bool marker=true,front=true;float imagePosition=-.12f,span=.3f;
bool cameraReflectorStripsFound(){return marker;}
float cameraReflectorStripsHorizontalError(){return imagePosition;}
float cameraReflectorStripsHorizontalSpan(){return span;}
float cameraReflectorStripsConfidence(){return .98f;}
#define SND_WARNING 1
#define DOCK_SLIP_RECOVERY true
#define DOCK_REFLECTOR_STRIPS_ANGULAR_P .7f
#define DOCK_REFLECTOR_STRIPS_MAX_ANGULAR .35f
#define DOCK_REFLECTOR_STRIPS_MAX_ERROR_FOR_DRIVE .65f
#define DOCK_LINEAR_SPEED .02f
#define DOCK_FRONT_SIDE front
DockSlipRecovery dockSlipRecovery;
float linear=0,angular=0;bool mow=true;
void track(){bool runControl=true;
#include "tracking-production.inc"
}
void reset(bool undock,bool frontDock){
 dockSlipRecovery.reset();maps.undock=undock;front=frontDock;
 stateEstimator.stateOp=undock?OP_MOW:OP_DOCK;
 stateEstimator.reflectorDockingGate.phase=undock?ReflectorDockingGate::Undock:ReflectorDockingGate::Dock;
 stateEstimator.stateDelta=0;marker=true;imagePosition=-.12f;span=.3f;bumper.hit=false;
 now=0;motor.motorLeftTicks=motor.motorRightTicks=10000;motor.stops=0;op.changes=0;
}
int main(){
 for(bool undock:{false,true})for(bool frontDock:{false,true}){
  reset(undock,frontDock);track();assert(!mow);
  const bool reverse=undock?frontDock:!frontDock;
  assert(reverse?linear<0:linear>0);
  now=12000;motor.motorRightTicks=reverse?9760:10240;track();
  assert(dockSlipRecovery.phase()==DockSlipRecovery::Settling && linear==0 && angular==0);
  now=12700;track();assert(dockSlipRecovery.phase()==DockSlipRecovery::Turning);
  now=15200;track();assert(linear==0 && std::fabs(angular-.20f)<.0001f);
  // Marker loss stops the actual production integration, then reacquires at rest.
  marker=false;now=15300;track();assert(linear==0 && angular==0);
  marker=true;now=15400;track();assert(dockSlipRecovery.phase()==DockSlipRecovery::Settling);
  now=16100;track();now=16300;stateEstimator.stateDelta=.11f;track();
  assert(dockSlipRecovery.phase()==DockSlipRecovery::Tracking && linear==0 && angular==0);
  now=16400;track();assert(reverse?linear<0:linear>0);
  bumper.hit=true;track();assert(dockSlipRecovery.phase()==DockSlipRecovery::Failed && linear==0 && angular==0 && op.changes==1);
 }
 // Merely being in Mow must not authorize camera recovery off the dock route.
 reset(true,true);maps.undock=false;track();now=12000;motor.motorRightTicks=9760;track();
 assert(dockSlipRecovery.phase()==DockSlipRecovery::Tracking);
 // Blocked/acquiring gates remain stopped even with valid markers and ticks.
 for(auto phase:{ReflectorDockingGate::Blocked,ReflectorDockingGate::AcquireUndock}){
  reset(true,true);stateEstimator.reflectorDockingGate.phase=phase;track();now=12000;motor.motorRightTicks=9760;track();
  assert(linear==0 && angular==0 && dockSlipRecovery.phase()==DockSlipRecovery::Tracking);
 }
 puts("PASS production camera slip recovery: docking/undocking, front/rear dock, reverse encoder motion, bounded pivot, marker pause, bumper and blocked gate");
}
