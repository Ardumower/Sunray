#include "src/navigation/UndockGpsSteering.h"
#include "src/navigation/DockMotionLimit.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
constexpr float PI=3.14159265358979323846f;
constexpr float TARGET_ANGLE_TOLERANCE=20;
constexpr float DOCK_LINEAR_SPEED=.02f;
#define DOCK_APPROACH_MAX_WHEEL_SPEED .10f
struct {float lateralError=0;} stateEstimator;
struct {float linearSpeedSet=0;} motor;
float steering(bool gpsUndocking,float trackerDiffDelta,float linear,float p=1.1f,float k=.1f){
 float angular;
#include "undock-steering-production.inc"
 return angular;
}
float speed(bool gpsUndocking){
 float linear;
#include "undock-speed-production.inc"
 return linear;
}
float wrap(float a){return std::atan2(std::sin(a),std::cos(a));}
// Differential-drive integration from the recorded live pose and exit segment.
// Include the actual alignment switch, ramp and wheel cap that caused the cycle.
float replay(bool fixed,bool mirror,bool reverse){
 float x=10.44f,y=-1.04f,yaw=-3.08f;
 const float ax=10.8839f,ay=-.7783f,tx=11.73f,ty=-1.31f;
 if(!reverse)yaw=wrap(yaw+PI);
 motor.linearSpeedSet=0;
 for(int step=0;step<6000;++step){
  const float distance=std::hypot(tx-x,ty-y);
  if(distance<.1f)return distance;
  const float error=wrap(std::atan2(ty-y,tx-x)+(reverse?PI:0)-yaw);
  stateEstimator.lateralError=((ty-ay)*x-(tx-ax)*y+(tx*ay-ty*ax))/std::hypot(tx-ax,ty-ay);
  if(mirror)stateEstimator.lateralError=-stateEstimator.lateralError;
  float linear=0,angular;
  if(std::fabs(error)>20*PI/180) angular=std::copysign(29*PI/180,error);
  else {
   linear=speed(fixed);
   angular=steering(fixed,mirror?-error:error,linear);
   if(mirror)angular=-angular;
   if(reverse)linear=-linear;
  }
  linear=.9f*motor.linearSpeedSet+.1f*linear;
  limitDockWheelSpeed(linear,angular,.48f,.1f);
  assert(std::fabs(linear)+.24f*std::fabs(angular)<=.100001f);
  motor.linearSpeedSet=linear;
  yaw=wrap(yaw+angular*.05f);x+=linear*std::cos(yaw)*.05f;y+=linear*std::sin(yaw)*.05f;
 }
 return std::hypot(tx-x,ty-y);
}
int main(){
 assert(speed(false)==.02f);assert(speed(true)==.1f);
 stateEstimator.lateralError=.46f;motor.linearSpeedSet=-.007f;
 assert(steering(false,-.30f,.02f)>0); // recorded wrong-way turn
 assert(steering(true,-.30f,.1f)<0); // align towards the target
 for(float e:{-.349f,.349f})for(float lateral:{-2.f,-.46f,0.f,.46f,2.f}){
  stateEstimator.lateralError=lateral;
  assert(steering(true,e,.1f)*e>0); // path correction cannot cross the rotation threshold
 }
 const float oldDistance=replay(false,false,true);
 assert(oldDistance>1.f); // reproduce the old limit cycle without reaching the target
 for(bool mirror:{false,true})for(bool reverse:{false,true})assert(replay(true,mirror,reverse)<.1f);
 std::printf("PASS GPS exit: recorded oscillation reproduced (%.2fm remaining), corrected forward/reverse and mirrored paths reach target; wheel cap retained\n",oldDistance);
}
