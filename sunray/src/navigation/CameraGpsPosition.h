#pragma once
#include <cmath>

// RTK corrects the map position, never camera steering or IMU yaw. Reacquire
// after three consecutive plausible solutions of the same quality; bound corrections so one fix cannot
// teleport the robot across the camera exit radius. Units: metres/milliseconds.
class CameraGpsPosition {
public:
  enum Quality { Invalid, Float, Fixed };
private:
  Quality quality = Invalid;
  unsigned long lastTime = 0;
  float lastX = 0, lastY = 0;
  unsigned int fixes = 0;
public:
  void reset() { fixes = 0; quality = Invalid; }
  bool ready(unsigned long now) const { return fixes >= 3 && now-lastTime <= 1000; }
  Quality activeQuality(unsigned long now) const { return ready(now) ? quality : Invalid; }
  bool update(Quality incoming, float gpsX, float gpsY, unsigned long now, float &x, float &y) {
    if ((incoming != Float && incoming != Fixed) || !std::isfinite(gpsX) || !std::isfinite(gpsY) ||
        !std::isfinite(x) || !std::isfinite(y)) { reset(); return false; }
    const unsigned long elapsed = now-lastTime;
    const float step = std::hypot(gpsX-lastX, gpsY-lastY);
    const float error = std::hypot(gpsX-x, gpsY-y);
    if (error > 2.0f) { reset(); return false; }
    if (!fixes || incoming != quality || elapsed > 1000 || step > 0.15f + 0.5f*elapsed/1000.0f) fixes = 1;
    else if (elapsed == 0) return false;
    else if (fixes < 3) ++fixes;
    quality = incoming;
    lastTime = now; lastX = gpsX; lastY = gpsY;
    if (fixes < 3) return false;
    // Float receives a smaller, time-based gain and correction speed.
    // Fix: max 10 cm/s; float: max 5 cm/s, 1 s smoothing time constant.
    const float dt = (elapsed < 250UL ? elapsed : 250UL)/1000.0f;
    const float limit = (incoming == Fixed ? 0.1f : 0.05f)*dt;
    const float weight = incoming == Fixed ? 1.0f : 1.0f-std::exp(-dt);
    const float gain = error*weight > limit ? limit/error : weight;
    x += gain*(gpsX-x); y += gain*(gpsY-y);
    return true;
  }
};
