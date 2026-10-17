#pragma once

#ifdef __linux__

#include <atomic>
#include <cstdint>

struct ReflectorStripsObservation {
  bool found{false};
  float horizontalError{0.0f}; // -1 left, +1 right
  float confidence{0.0f};
  int stripCount{0};
  float verticalSpan{0.0f};    // fraction of image height
  unsigned long ageMs{0};
};

class ReflectorStripsDetector {
public:
  static ReflectorStripsDetector& instance();

  void processRgb(const uint8_t* rgb, int width, int height, unsigned long timestampMs);
  ReflectorStripsObservation observation(unsigned long nowMs) const;
  void clear();

private:
  ReflectorStripsDetector() = default;

  int acquireCount_{0}; // written only by the camera capture thread
  std::atomic<bool> found_{false};
  std::atomic<float> horizontalError_{0.0f};
  std::atomic<float> confidence_{0.0f};
  std::atomic<int> stripCount_{0};
  std::atomic<float> verticalSpan_{0.0f};
  std::atomic<unsigned long> timestampMs_{0};
};

#endif
