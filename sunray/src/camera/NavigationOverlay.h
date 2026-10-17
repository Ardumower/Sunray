#pragma once
// Snapshot published by the control loop; camera rendering never reads robot state.
namespace NavigationOverlay {
enum Mode { Hidden, RtkFix, RtkFloat, GpsWait, Camera, CameraWait, CameraSearch, CameraStop, Odometry, Guidance };
inline const char* label(Mode mode) {
  switch (mode) {
    case RtkFix: return "RTK FIX";
    case RtkFloat: return "RTK FLOAT";
    case GpsWait: return "GPS WAIT";
    case Camera: return "CAM";
    case CameraWait: return "CAM WAIT";
    case CameraSearch: return "CAM SEARCH";
    case CameraStop: return "CAM STOP";
    case Odometry: return "ODO";
    case Guidance: return "GUIDE";
    default: return "";
  }
}
}
extern "C" int cameraGetNavigationMode();
extern "C" void cameraUpdateNavigationOverlay();
