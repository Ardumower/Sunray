#pragma once
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

// Map source-frame age to current wall time so NTP steps/slews are reflected.
// Delayed analysis retains its capture age, never the current send time.
class CameraFrameClock {
public:
  CameraFrameClock()
      : CameraFrameClock(monotonicMs(), std::chrono::system_clock::now()) { live_=true; }
  CameraFrameClock(uint64_t monotonic, std::chrono::system_clock::time_point wall)
      : monotonic_(monotonic), wallMs_(std::chrono::duration_cast<
            std::chrono::milliseconds>(wall.time_since_epoch()).count()) {}

  static uint64_t monotonicMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::string label(uint64_t frameMs) const {
    if(live_)return labelAt(frameMs,monotonicMs(),std::chrono::system_clock::now());
    return labelAt(frameMs,monotonic_,std::chrono::system_clock::time_point(std::chrono::milliseconds(wallMs_)));
  }

  static std::string labelAt(uint64_t frameMs,uint64_t nowMs,std::chrono::system_clock::time_point wall){
    const int64_t epochMs = std::chrono::duration_cast<std::chrono::milliseconds>(wall.time_since_epoch()).count() + int64_t(frameMs) - int64_t(nowMs);
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
  bool live_=false;
  uint64_t monotonic_;
  int64_t wallMs_;
};
