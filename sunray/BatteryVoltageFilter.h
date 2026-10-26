#pragma once
#include <cmath>
#include <stdint.h>

// Time-based low-pass for the return-to-dock decision, independent of the
// faster voltage used for display, charging and undervoltage protection.
class BatteryVoltageFilter {
public:
  void reset() { initialized_ = false; }
  float update(float voltage, uint32_t now, uint32_t timeConstantMs) {
    // Do not hide a missing/invalid supply measurement behind old good values.
    // Re-seed when valid measurements return, including during startup.
    if (!std::isfinite(voltage) || voltage <= 0) {
      reset();
      return voltage;
    }
    if (!initialized_ || timeConstantMs == 0) {
      value_ = voltage;
      initialized_ = true;
    } else {
      const uint32_t elapsed = now - lastTime_;
      const double alpha = -std::expm1(-double(elapsed) / timeConstantMs);
      value_ += alpha * (voltage - value_);
    }
    lastTime_ = now;
    return float(value_);
  }
private:
  bool initialized_ = false;
  uint32_t lastTime_ = 0;
  double value_ = 0;
};
