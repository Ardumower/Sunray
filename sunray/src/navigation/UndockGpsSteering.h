#pragma once
#include <cmath>

// The camera/GPS handoff can leave a lateral offset from the exit segment.
// At very low speed an unbounded Stanley correction steers away from the
// target until the rotate-in-place branch reverses it, repeating indefinitely.
inline float undockGpsSteering(float headingError, float lateralError,
                              float requestedSpeed, float p, float k,
                              float alignmentTolerance) {
  const float correction = std::atan2(k * lateralError, .001f + std::fabs(requestedSpeed));
  // Keep the steady-state heading error inside half the alignment tolerance.
  // Do not feed the previous wheel-limited speed back into this correction:
  // stronger steering would otherwise reduce speed and amplify itself.
  const float limit = std::fabs(p * alignmentTolerance) * .5f;
  return p * headingError + (correction < -limit ? -limit : correction > limit ? limit : correction);
}
