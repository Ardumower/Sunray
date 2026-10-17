#pragma once
#include <cmath>

// Estimate body heading from a straight RTK track, including reverse driving.
// Camera steering never reads this estimator. Wheel travel prevents stationary
// GPS drift from being mistaken for a heading measurement.
class CameraGpsHeading {
  bool started = false;
  int direction = 0;
  unsigned int samples = 0;
  unsigned long lastTime = 0;
  float originX=0, originY=0, lastX=0, lastY=0, originYaw=0, originTravel=0;
  static float wrap(float a) { return std::atan2(std::sin(a),std::cos(a)); }
public:
  void reset() { started=false; samples=0; }
  bool update(bool trustedFix, float x, float y, float travel, float speed,
              float turnRate, unsigned long now, float &yaw) {
    if (!trustedFix || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(travel) || !std::isfinite(yaw) || !std::isfinite(speed) ||
        !std::isfinite(turnRate) || std::fabs(speed)<0.005f || std::fabs(turnRate)>0.08f) {
      reset(); return false;
    }
    const int nextDirection=speed<0 ? -1 : 1;
    const unsigned long elapsed=now-lastTime;
    if (started && (nextDirection!=direction || elapsed>1000 ||
        std::hypot(x-lastX,y-lastY)>0.15f+0.5f*elapsed/1000.0f ||
        std::fabs(wrap(yaw-originYaw))>0.175f || travel<originTravel)) reset();
    if (!started) {
      started=true; direction=nextDirection; samples=1;
      originX=x; originY=y; originYaw=yaw; originTravel=travel;
      lastX=x; lastY=y; lastTime=now; return false;
    }
    if (!elapsed) return false;
    lastX=x; lastY=y; lastTime=now; ++samples;
    const float displacement=std::hypot(x-originX,y-originY);
    const float wheels=travel-originTravel;
    if (displacement<0.25f || wheels<0.20f || samples<5) return false;
    if (displacement<0.4f*wheels || displacement>1.8f*wheels) { reset(); return false; }
    const float heading=wrap(std::atan2(y-originY,x-originX)+(direction<0 ? 3.14159265f : 0));
    const float error=wrap(heading-yaw);
    const float limit=0.52359878f; // at most 30 degrees per independent track
    yaw=wrap(yaw+(error>limit ? limit : error< -limit ? -limit : error));
    reset(); return true;
  }
};
