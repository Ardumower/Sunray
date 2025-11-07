// Small accessor shims to avoid including Arduino/robot headers in CameraStreamer
#include "../../robot.h"

extern "C" float cameraGetLinearSet() {
  return motor.linearSpeedSet;
}

extern "C" float cameraGetAngularSet() {
  return motor.angularSpeedSet;
}

