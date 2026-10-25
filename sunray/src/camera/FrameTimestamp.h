#pragma once
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

// One stable wall/monotonic mapping per capture session. Reusing an analysis
// result must retain its source frame's time, rather than its latest send time.
class CameraFrameClock {
public:
  CameraFrameClock()
      : CameraFrameClock(monotonicMs(), std::chrono::system_clock::now()) {}
  CameraFrameClock(uint64_t monotonic, std::chrono::system_clock::time_point wall)
      : monotonic_(monotonic), wallMs_(std::chrono::duration_cast<
            std::chrono::milliseconds>(wall.time_since_epoch()).count()) {}

  static uint64_t monotonicMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::string label(uint64_t frameMs) const {
    const int64_t epochMs = wallMs_ + int64_t(frameMs) - int64_t(monotonic_);
    std::time_t seconds = epochMs / 1000;
    int fraction = int(epochMs % 1000);
    if (fraction < 0) { --seconds; fraction += 1000; }
    std::tm local{};
    if (!localtime_r(&seconds, &local)) return "00:00:00.000";
    char text[24];
    std::snprintf(text, sizeof(text), "%02d:%02d:%02d.%03d",
                  local.tm_hour, local.tm_min, local.tm_sec, fraction);
    return text;
  }

private:
  uint64_t monotonic_;
  int64_t wallMs_;
};
