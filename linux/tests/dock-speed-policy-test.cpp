#include "src/navigation/DockMotionLimit.h"
#include <cassert>
#include <cstdio>
#define DOCK_RTK_SPEED .5f
#define DOCK_MAX_WHEEL_SPEED .02f
#define DOCK_APPROACH_MAX_WHEEL_SPEED .10f
const int OP_DOCK=4,OP_MOW=1,OP_IDLE=0,LOC_REFLECTOR_STRIPS=5,LOC_GPS=0;
struct {int stateOp=OP_DOCK,stateLocalizationMode=LOC_GPS;} stateEstimator;
struct {bool exit=false;bool isUndocking(){return exit;}} maps;
float linearSpeedSet,angularSpeedSet,wheelBaseCm=48;
void apply(){
#include "dock-speed-production.inc"
}
void check(float expected){
 linearSpeedSet=.6;angularSpeedSet=1;apply();
 const float peak=std::fabs(linearSpeedSet)+std::fabs(angularSpeedSet)*.24f;
 assert(std::fabs(peak-expected)<.000001f);
 assert(std::fabs(linearSpeedSet-angularSpeedSet*.6f)<.000001f);
}
int main(){
 check(.5f);stateEstimator.stateLocalizationMode=LOC_REFLECTOR_STRIPS;check(.02f);
 stateEstimator.stateOp=OP_MOW;maps.exit=true;check(.02f);
 stateEstimator.stateLocalizationMode=LOC_GPS;check(.1f);
 maps.exit=false;check(.84f);stateEstimator.stateOp=OP_IDLE;check(.84f);
 std::puts("PASS production wheel limits: RTK docking 50cm/s, GPS exit 10cm/s, camera 2cm/s, ordinary mowing unchanged");
}
