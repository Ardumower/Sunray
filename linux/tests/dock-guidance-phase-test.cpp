#include "src/navigation/ReflectorDockingGate.h"
#include "src/navigation/DockMarkerSearch.h"
#include <cassert>
#include <initializer_list>
#include <cstdio>
#define DOCK_REFLECTOR_STRIPS 1
enum {LOC_GPS,LOC_REFLECTOR_STRIPS,LOC_IMU_ODO_ONLY};
struct {ReflectorDockingGate reflectorDockingGate;int stateLocalizationMode=LOC_GPS;} stateEstimator;
struct DockOp {DockMarkerSearch cameraSearch;bool dockContactAdvanceActive=false;bool cameraGuidanceActive()const;};
#include "dock-guidance-production.inc"
int main(){
 DockOp dock;auto& gate=stateEstimator.reflectorDockingGate;
 for(auto p:{gate.Inactive,gate.Approach,gate.Complete}){gate.phase=p;assert(!dock.cameraGuidanceActive());}
 for(auto p:{gate.AcquireDock,gate.Dock,gate.AcquireUndock,gate.Undock,gate.Blocked}){gate.phase=p;assert(dock.cameraGuidanceActive());}
 gate.reset();dock.cameraSearch.start(0,0);assert(dock.cameraGuidanceActive());dock.cameraSearch.reset();
 stateEstimator.stateLocalizationMode=LOC_REFLECTOR_STRIPS;assert(dock.cameraGuidanceActive());
 stateEstimator.stateLocalizationMode=LOC_IMU_ODO_ONLY;dock.dockContactAdvanceActive=true;assert(dock.cameraGuidanceActive());
 dock.dockContactAdvanceActive=false;stateEstimator.stateLocalizationMode=LOC_GPS;gate.phase=gate.Complete;assert(!dock.cameraGuidanceActive());
 std::puts("PASS production docking/undocking phase: GPS approach/exit permit avoidance; acquisition, camera, search and contact preserve stop");
}
