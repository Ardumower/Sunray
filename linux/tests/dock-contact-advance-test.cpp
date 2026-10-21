#include "src/navigation/ReflectorDockingGate.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <initializer_list>
using std::abs;
#define DOCK_CONTACT_ADVANCE_DISTANCE .10f
#define DOCK_LINEAR_SPEED .02f
#define DOCK_FRONT_SIDE true
#define DOCK_REFLECTOR_STRIPS
#define SENS_BUMPER 13
#define OP_DOCK 4
unsigned long now=1000;unsigned long millis(){return now;}
bool pressed=false,marker=false;int markerChecks=0,searchChecks=0;
struct {bool obstacle(){return pressed;}} bumper;
struct {unsigned long motorLeftTicks=1000,motorRightTicks=1000;float ticksPerCm=10,linear=0,angular=0;int stops=0;
 void stopImmediately(bool){linear=angular=0;++stops;}
 void setLinearAngularSpeed(float l,float a,bool){linear=l;angular=a;}
 void enableTractionMotors(bool){}
} motor;
struct {bool docked=false;void setIsDocked(bool d){docked=d;}void resetIdle(){}} battery;
struct {template<class T>void print(T){} template<class T>void println(T){}} CONSOLE;
struct {int stateOp=OP_DOCK;ReflectorDockingGate reflectorDockingGate;} stateEstimator;
struct {bool isDocking(){return true;}} maps;
struct {int calls=0;void trackLine(bool){++calls;}} lineTracker;
bool cameraReflectorStripsFound(){++markerChecks;return marker;}
int chargeOp=1,errorOp=2;
struct Op {void onChargerConnected(){battery.setIsDocked(true);}};
struct DockOp:Op {
 bool dockContactAdvanceActive=false,shouldStop=false,cameraSearchAvailable=true;
 unsigned long dockContactAdvanceStartLeftTicks=0,dockContactAdvanceStartRightTicks=0,dockContactAdvanceStopTime=0;
 int changed=0;
 void run();void onChargerConnected();
 bool stopOnDockingObstacle(int){motor.stopImmediately(true);changed=errorOp;shouldStop=true;return true;}
 bool runCameraSearch(){++searchChecks;return false;}
 bool startCameraSearch(bool){++searchChecks;return true;}
 bool detectObstacle(){return false;}void detectObstacleRotation(){}void detectSensorMalfunction(){}
 void changeOp(int op){changed=op;}
} dockOp;
#include "dock-contact-production.inc"
void estimatorPhaseStop(){
 const auto stateOp=stateEstimator.stateOp;
 auto& reflectorDockingGate=stateEstimator.reflectorDockingGate;
#include "dock-contact-estimator.inc"
}
int main(){
 dockOp.onChargerConnected();assert(dockOp.dockContactAdvanceActive);
 auto deadline=dockOp.dockContactAdvanceStopTime;
 for(auto phase:{ReflectorDockingGate::Inactive,ReflectorDockingGate::AcquireDock,ReflectorDockingGate::Blocked,ReflectorDockingGate::Dock}){
  stateEstimator.reflectorDockingGate.phase=phase;
  int stops=motor.stops;estimatorPhaseStop();dockOp.run();
  assert(motor.linear==.02f&&motor.angular==0&&motor.stops==stops);
  assert(markerChecks==0&&searchChecks==0&&lineTracker.calls==0);
 }
 // Contact bounce does not reset odometry or extend the original deadline.
 now+=1000;motor.motorLeftTicks+=50;motor.motorRightTicks+=50;
 dockOp.onChargerConnected();assert(dockOp.dockContactAdvanceStopTime==deadline&&dockOp.dockContactAdvanceStartLeftTicks==1000);
 dockOp.run();assert(!dockOp.changed&&motor.linear>0);
 pressed=true;dockOp.run();assert(dockOp.changed==errorOp&&motor.linear==0);pressed=false;
 dockOp.changed=0;dockOp.shouldStop=false;
 motor.motorLeftTicks+=51;motor.motorRightTicks+=51;dockOp.run();
 assert(dockOp.changed==chargeOp&&!dockOp.dockContactAdvanceActive&&battery.docked&&motor.linear==0);
 // Fresh bounded advance with stalled odometry still stops at its deadline.
 dockOp.changed=0;dockOp.onChargerConnected();now=dockOp.dockContactAdvanceStopTime;dockOp.run();
 assert(dockOp.changed==chargeOp&&!dockOp.dockContactAdvanceActive&&motor.linear==0);
 // Outside contact advance, camera loss retains the estimator stop gate.
 stateEstimator.reflectorDockingGate.phase=ReflectorDockingGate::AcquireDock;
 int stops=motor.stops;estimatorPhaseStop();assert(motor.stops==stops+1);
 dockOp.dockContactAdvanceActive=true;stateEstimator.stateOp=0;stops=motor.stops;
 estimatorPhaseStop();assert(motor.stops==stops+1);
 std::puts("PASS first contact: marker-independent straight advance, 10cm bound, timeout, bumper, contact bounce and ordinary camera loss stop");
}
