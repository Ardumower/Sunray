#include "src/navigation/DockMarkerSearch.h"
#include "src/navigation/DockMotionLimit.h"
#include <cassert>
#include <cstdio>
#include <limits>
#define DOCK_REFLECTOR_STRIPS 1
#define DOCK_MAX_WHEEL_SPEED .02f
#define SENS_BUMPER 13
unsigned long now=0;
unsigned long millis(){return now;}
struct {float stateDelta=0;} stateEstimator;
struct {void resetIdle(){}} battery;
struct {void println(const char*){}} CONSOLE;
struct {
 float wheelBaseCm=48,lin=0,ang=0;int commands=0;
 void stopImmediately(bool){lin=ang=0;}
 void setMowState(bool on){assert(!on);}
 void enableTractionMotors(bool){}
 void setLinearAngularSpeed(float l,float a,bool ramp){assert(!ramp);lin=l;ang=a;++commands;}
} motor;
bool obstacle=false,bumperPressed=false,sensorFailure=false;
struct {bool obstacle(){return bumperPressed;}} bumper;
bool detectObstacle(){if(obstacle)motor.stopImmediately(true);return obstacle;}
int idleOp;
struct DockOp {
 DockMarkerSearch cameraSearch;bool shouldStop=false,entry=false;int entryChecks=0,routeCalls=0;
 bool tryCameraEntry(bool){++entryChecks;if(entry)motor.stopImmediately(true);return entry;}
 void detectSensorMalfunction(){if(sensorFailure){shouldStop=true;motor.stopImmediately(true);}}
 void changeOp(int&,bool){shouldStop=true;}
 bool stopOnDockingObstacle(int){motor.stopImmediately(true);shouldStop=true;return true;}
 bool runCameraSearch();
 void prefix(){
#include "dock-search-prefix.inc"
 ++routeCalls;
 }
};
#include "dock-search-production.inc"
int main(){
 assert(DockMarkerSearch::eligible(2,2,0,0,4,0));
 assert(!DockMarkerSearch::eligible(2,2.01f,0,0,4,0));
 assert(!DockMarkerSearch::eligible(-.01f,0,0,0,4,0));
 assert(!DockMarkerSearch::eligible(4.01f,0,0,0,4,0));
 assert(!DockMarkerSearch::eligible(0,0,0,0,0,0));
 DockMarkerSearch scan;scan.start(3.0f,0);
 assert(scan.update(3.0f,499)==scan.Settling);
 assert(scan.update(3.0f,500)==scan.Turning);
 // One full measured turn across the +/- pi boundary, at 2 cm/s wheel speed.
 for(int i=1;i<=629;++i){const float yaw=std::atan2(std::sin(3.f+i*.01f),std::cos(3.f+i*.01f));
   scan.update(yaw,500+i*120);if(i<629)assert(scan.active());}
 assert(scan.state==scan.Finished);
 scan.start(0,0);scan.update(0,500);assert(scan.update(0,15500)==scan.Stalled);
 scan.start(0,0);scan.update(0,500);assert(scan.update(1,520)==scan.Stalled);
 scan.start(0,0);assert(scan.update(0,300000)==scan.Stalled);
 scan.start(std::numeric_limits<float>::quiet_NaN(),0);assert(scan.state==scan.Stalled);
 DockOp op;now=0;op.cameraSearch.start(0,now);op.prefix();assert(motor.ang==0&&op.routeCalls==0);
 now=500;op.prefix();assert(motor.lin==0&&motor.ang>0&&motor.ang*.24f<=.020001f);
 int commands=motor.commands;op.entry=true;now=600;op.prefix();
 assert(!op.cameraSearch.active()&&motor.ang==0&&motor.commands==commands);
 op.entry=false;op.cameraSearch.start(0,now);bumperPressed=true;
 int checks=op.entryChecks;op.prefix();assert(op.shouldStop&&motor.ang==0&&op.entryChecks==checks);
 bumperPressed=false;op.shouldStop=false;obstacle=true;op.prefix();assert(op.entryChecks==checks);
 obstacle=false;op.shouldStop=false;now=1100;op.prefix();
 now=16100;op.prefix();assert(op.shouldStop&&motor.ang==0&&op.routeCalls==0);
 std::puts("PASS dock search: station-side geometry, 360-degree bound, yaw wrap/stall/jump, zero translation, wheel cap, marker and bumper priority");
}
