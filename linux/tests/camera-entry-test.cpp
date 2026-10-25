#include "src/navigation/ReflectorDockingGate.h"
#include "src/navigation/DockMarkerSearch.h"
DockMarkerSearch cameraSearch;
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
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
enum {SOL_INVALID,SOL_FLOAT,SOL_FIXED};
struct {int solution=SOL_FIXED;} gps;
struct {bool connected=false;bool chargerConnected(){return connected;}} battery;
bool initiatedByOperator=true;
int* previousOp=&idleOp;
bool marker=true;
float imageError=0;
float cameraReflectorStripsHorizontalError(){return imageError;}
bool cameraReflectorStripsFound(){return marker;}
bool dock(){
#include "dock-entry-production.inc"
 return automaticCameraEntry;
}
bool undock(){
#include "undock-entry-production.inc"
 return resumeCamera;
}
bool clearsStaleRoute(){
#include "undock-entry-production.inc"
 return clearStaleDockRoute;
}
int main(){
 assert(dock());assert(undock());
 // Both commands accept a marker up to 2 m from the final path, not just
 // a circle around a waypoint. The earlier approach segment is excluded.
 stateEstimator.stateY=2;assert(dock());assert(undock());
 stateEstimator.stateY=2.01f;assert(!dock());assert(!undock());
 stateEstimator.stateY=1.5f;marker=false;assert(!dock());assert(!undock());
 marker=true;stateEstimator.stateDelta=PI;assert(dock());assert(undock());
 // Marker orientation takes precedence over drifting map yaw, even at edge.
 imageError=-.88f;assert(dock());assert(undock());
 imageError=1.01f;assert(!dock());assert(!undock());
 imageError=std::numeric_limits<float>::quiet_NaN();assert(!dock());assert(!undock());
 imageError=0;stateEstimator.stateDelta=0;stateEstimator.stateX=-3;stateEstimator.stateY=0;
 assert(!dock());assert(!undock());stateEstimator.stateX=2;
 // Regression: the 2 m endpoint circle includes locations before the handoff.
 // Even a stable marker there must not skip the GPS approach at command entry.
 for(float x:{-1.5f,-1.0f,-0.01f}) {
   stateEstimator.stateX=x;assert(!dock());assert(!undock());
   maps.wayMode=WAY_DOCK;
   assert(!undock());assert(clearsStaleRoute());
   gps.solution=SOL_FLOAT;assert(!clearsStaleRoute());gps.solution=SOL_FIXED;
   battery.connected=true;assert(!clearsStaleRoute());battery.connected=false;
   maps.wayMode=0;assert(!clearsStaleRoute());
 }
 // A scan authorized by the route gate within the 1 m tolerance can acquire.
 stateEstimator.stateX=-0.5f;cameraSearch.start(0,0);assert(dock());cameraSearch.reset();
 stateEstimator.stateX=0;assert(dock());stateEstimator.stateX=2;
 // Command entry is supported after Idle, Error and calibration.
 previousOp=&otherOp;assert(dock());previousOp=&imuCalibrationOp;assert(dock());
 previousOp=&idleOp;
 maps.valid=false;assert(!dock());assert(!undock());maps.valid=true;
 maps.dockPoints.numPoints=1;assert(!dock());assert(!undock());maps.dockPoints.numPoints=3;
 stateEstimator.stateX=std::numeric_limits<float>::quiet_NaN();assert(!dock());assert(!undock());
 stateEstimator.stateX=2;stateEstimator.stateDelta=std::numeric_limits<float>::quiet_NaN();assert(dock());assert(undock());
 stateEstimator.stateDelta=0;stateEstimator.stateX=1;
 assert(dock());assert(!undock()); // already at 1 m handoff: do not restart camera exit
 stateEstimator.stateX=1.01f;assert(undock());
 marker=false;assert(!dock());assert(undock()); // narrow corridor still acquires while stopped
 // Genuine station-side exits retain their camera guidance, with stale WAY_DOCK too.
 maps.wayMode=WAY_DOCK;stateEstimator.stateX=2;assert(undock());assert(!clearsStaleRoute());maps.wayMode=0;
 marker=true;frontDock=false;stateEstimator.stateDelta=PI;
 assert(dock());assert(undock());
 stateEstimator.stateDelta=0;assert(dock());assert(undock());
 frontDock=true;
 // Automatic entry uses the same standstill and loss gate as explicit entry.
 ReflectorDockingGate gate;gate.confirmDockEntry(100);
 gate.update(true,false,true,2,0,0,0,0,true,599,1,1,500);assert(!gate.mayDrive());
 gate.update(true,false,true,2,0,0,0,0,true,600,1,1,500);assert(gate.mayDrive());
 gate.update(true,false,true,2,0,0,0,0,false,620,1,1,500);assert(!gate.mayDrive());
 std::puts("PASS production Dock/Mow camera entry: 2m path distance, image orientation, stable marker, reentry, handoff and stop gate");
}
