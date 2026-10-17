#include "src/navigation/ReflectorDockingGate.h"
#include <cassert>
#include <cstdio>
#include <limits>
#define PI 3.14159265358979323846f
#define DOCK_FRONT_SIDE frontDock
bool frontDock=true;
#define DOCK_REFLECTOR_STRIPS_HANDOFF_RADIUS 1.0f
const int WAY_DOCK=1;
struct Point { float px,py; float x()const{return px;}float y()const{return py;} };
struct Map {
 struct { int numPoints=3; Point points[3]={{-4,0},{0,0},{4,0}}; } dockPoints;
 int wayMode=0;
 bool valid=true;
 bool getDockingPos(float& x,float& y,float& heading){x=4;y=0;heading=0;return valid;}
} maps;
struct {float stateX=2,stateY=0,stateDelta=0;} stateEstimator;
int idleOp,otherOp,imuCalibrationOp;
bool initiatedByOperator=true;
int* previousOp=&idleOp;
bool marker=true;
bool cameraReflectorStripsFound(){return marker;}
bool dock(){
#include "dock-entry-production.inc"
 return automaticCameraEntry;
}
bool undock(){
#include "undock-entry-production.inc"
 return resumeCamera;
}
int main(){
 assert(dock());assert(undock());
 // Both commands accept a marker up to 2 m from the final path, not just
 // a circle around a waypoint. The earlier approach segment is excluded.
 stateEstimator.stateY=2;assert(dock());assert(undock());
 stateEstimator.stateY=2.01f;assert(!dock());assert(!undock());
 stateEstimator.stateY=1.5f;marker=false;assert(!dock());assert(!undock());
 marker=true;stateEstimator.stateDelta=PI;assert(!dock());assert(!undock());
 stateEstimator.stateDelta=.79f;assert(!dock());assert(!undock());
 stateEstimator.stateDelta=.78f;assert(dock());assert(undock());
 stateEstimator.stateDelta=0;stateEstimator.stateX=-3;stateEstimator.stateY=0;
 assert(!dock());assert(!undock());
 stateEstimator.stateX=2;previousOp=&otherOp;assert(!dock());previousOp=&idleOp;
 // Dock pressed during startup calibration must use the same camera entry.
 previousOp=&imuCalibrationOp;assert(dock());
 initiatedByOperator=false;assert(!dock());
 initiatedByOperator=true;marker=false;assert(!dock());marker=true;
 previousOp=&idleOp;
 maps.valid=false;assert(!dock());assert(!undock());maps.valid=true;
 maps.dockPoints.numPoints=1;assert(!dock());assert(!undock());maps.dockPoints.numPoints=3;
 stateEstimator.stateX=std::numeric_limits<float>::quiet_NaN();assert(!dock());assert(!undock());
 stateEstimator.stateX=2;stateEstimator.stateDelta=std::numeric_limits<float>::quiet_NaN();assert(!dock());assert(!undock());
 stateEstimator.stateDelta=0;stateEstimator.stateX=1;
 assert(dock());assert(!undock()); // already at 1 m handoff: do not restart camera exit
 stateEstimator.stateX=1.01f;assert(undock());
 marker=false;assert(!dock());assert(undock()); // narrow corridor still acquires while stopped
 marker=true;frontDock=false;stateEstimator.stateDelta=PI;
 assert(dock());assert(undock());
 stateEstimator.stateDelta=0;assert(!dock());assert(!undock());
 frontDock=true;
 // Automatic entry uses the same standstill and loss gate as explicit entry.
 ReflectorDockingGate gate;gate.confirmDockEntry(100);
 gate.update(true,false,true,2,0,0,0,0,true,599,1,1,500);assert(!gate.mayDrive());
 gate.update(true,false,true,2,0,0,0,0,true,600,1,1,500);assert(gate.mayDrive());
 gate.update(true,false,true,2,0,0,0,0,false,620,1,1,500);assert(!gate.mayDrive());
 std::puts("PASS production Dock/Mow camera entry: 2m path distance, facing, stable marker, Idle, handoff and stop gate");
}
