#pragma once
#include <stdint.h>
#include <cmath>

// Conservative docking-only recovery. Encoder travel alone is never proof
// of progress. Detection uses camera geometry and IMU heading, never GPS.
class DockSlipRecovery {
public:
  enum Phase { Tracking, Settling, Turning, Failed };
  struct Input {
    uint32_t now;
    bool camera, bumper;
    float yaw, imageError, imageSpan, leftTravel, rightTravel;
  };
  void reset() { *this = DockSlipRecovery(); }
  Phase phase() const { return phase_; }
  void update(const Input& i, float& linear, float& angular) {
    if (phase_ == Failed) { linear=angular=0; return; }
    if (i.bumper) {
      phase_=Failed; linear=angular=0; return;
    }
    if (!i.camera) {
      // Missing or unstable markers pause motion without consuming a retry.
      cameraPaused_=true; baseline_=false; linear=angular=0; return;
    }
    if (cameraPaused_) {
      cameraPaused_=false;
      if (phase_==Settling || phase_==Turning) {
        phase_=Settling; phaseAt_=i.now; // resume with fresh gyro baseline
      }
    }
    if (phase_ == Settling) {
      linear=angular=0;
      if (uint32_t(i.now - phaseAt_) >= 700) {
        phase_=Turning; phaseAt_=i.now; turnYaw_=i.yaw; turnError_=i.imageError;
      }
      return;
    }
    if (phase_ == Turning) {
      linear=angular=0;
      // Bound the pivot using actual IMU rotation, not wheel odometry.
      // Stop at alignment before the 6-degree limit: otherwise the normal
      // tracker immediately steers back and can undo the escape manoeuvre.
      const float turned = angle(i.yaw, turnYaw_);
      const bool aligned = std::fabs(turnError_) > .05f && turned >= .025f &&
          (std::fabs(i.imageError) <= .025f || i.imageError*turnError_ < 0);
      if (aligned || turned >= 0.105f) {
        phase_=Tracking; baseline_=false; return;
      }
      if (uint32_t(i.now - phaseAt_) >= 5000) {
        // Slip alone never aborts docking. Pause before another bounded pivot.
        phase_=Settling; phaseAt_=i.now;
        direction_=std::fabs(i.imageError)>.025f ? (i.imageError>0 ? -1.f : 1.f) : -direction_;
        return;
      }
      const float ramp = std::fmin(1.0f, uint32_t(i.now-phaseAt_) / 2500.0f);
      angular=direction_*(0.08f + 0.12f*ramp); // bounded ramp to 0.20 rad/s
      return;
    }
    if (!i.camera || !std::isfinite(i.imageSpan) || i.imageSpan <= 0 || std::fabs(linear) < .005f) {
      baseline_=false; return;
    }
    if (!baseline_) { base_=i; baseline_=true; return; }
    // Marker position measures lateral movement; spacing measures approach.
    // Rotation is independently confirmed by the gyro. Never use brightness.
    const bool imageMoved = std::fabs(i.imageError-base_.imageError) > .03f ||
      std::fabs(i.imageSpan-base_.imageSpan) > std::fmax(.008f, base_.imageSpan*.04f);
    const bool rotated = angle(i.yaw,base_.yaw) > .04f;
    if (imageMoved || rotated) { base_=i; return; }
    if (uint32_t(i.now-base_.now) < 12000) return;
    const float wheelTravel=std::fmax(std::fabs(i.leftTravel-base_.leftTravel),
                                    std::fabs(i.rightTravel-base_.rightTravel));
    if (wheelTravel < .12f) return;
    linear=0;
    direction_ = angular < 0 ? -1.f : 1.f;
    angular=0;
    phase_=Settling; phaseAt_=i.now;
  }
private:
  static float angle(float a,float b) { return std::fabs(std::atan2(std::sin(a-b),std::cos(a-b))); }
  Phase phase_=Tracking;
  bool baseline_=false, cameraPaused_=false;
  uint32_t phaseAt_=0;
  float direction_=1, turnYaw_=0, turnError_=0;
  Input base_{};
};
