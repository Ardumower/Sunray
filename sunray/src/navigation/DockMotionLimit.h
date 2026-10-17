#pragma once
#include <cmath>
// Scale both commands together, preserving curvature and bounding both wheels.
inline void limitDockWheelSpeed(float& linear,float& angular,float wheelBaseM,float limit){
  const float peak=std::fabs(linear)+std::fabs(angular)*std::fabs(wheelBaseM)*.5f;
  if(limit<=0 || !std::isfinite(peak)){linear=angular=0;return;}
  if(peak>limit){const float scale=limit/peak;linear*=scale;angular*=scale;}
}
