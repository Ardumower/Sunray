#ifdef __linux__
#include <atomic>
#include "NavigationOverlay.h"
#endif
// Small accessor shims to avoid including Arduino/robot headers in CameraStreamer
#include "../../robot.h"
#ifdef __linux__
#include "../op/op.h"
#endif

extern "C" float cameraGetLinearSet() {
  return motor.linearSpeedSet;
}

extern "C" float cameraGetAngularSet() {
  return motor.angularSpeedSet;
}


#ifdef __linux__
namespace { std::atomic<int> navigationMode{NavigationOverlay::Hidden}; }
extern "C" int cameraGetNavigationMode() {
  return navigationMode.load(std::memory_order_relaxed);
}
extern "C" void cameraUpdateNavigationOverlay() {
  using namespace NavigationOverlay;
  Mode mode = Hidden;
  const auto op = activeOp->getGoalOperationType();
  if (op == OP_MOW || op == OP_DOCK) {
    const auto& gate = stateEstimator.reflectorDockingGate;
    if (op == OP_DOCK && dockOp.cameraSearch.active()) mode = CameraSearch;
    else if (gate.active()) {
      mode = gate.phase == ReflectorDockingGate::Blocked ? CameraStop :
          (gate.mayDrive() ? Camera : CameraWait);
    } else if (stateEstimator.stateLocalizationMode == LOC_REFLECTOR_STRIPS) mode = CameraWait;
    else if (stateEstimator.stateLocalizationMode == LOC_IMU_ODO_ONLY) mode = Odometry;
    else if (stateEstimator.stateLocalizationMode == LOC_GUIDANCE_SHEET) mode = Guidance;
    else mode = gps.solution == SOL_FIXED ? RtkFix : (gps.solution == SOL_FLOAT ? RtkFloat : GpsWait);
  }
  navigationMode.store(mode, std::memory_order_relaxed);
}
#endif
