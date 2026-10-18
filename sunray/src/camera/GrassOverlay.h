#pragma once
#ifdef __linux__
#include <cstdint>
#include <memory>
#include <vector>
// Asynchronous segmentation and optional front-camera slowdown observation.
// One pending frame and one completed frame; inference never blocks capture.
class GrassOverlay {
public:
  static bool obstaclePresent();
  static int obstacleState(); // -1 unavailable/stale, 0 clear, 1 obstacle
  GrassOverlay();
  ~GrassOverlay();
  void submit(const uint8_t* rgb, int width, int height, uint64_t timestamp);
  bool latest(std::vector<uint8_t>& rgb, uint64_t& timestamp, double& inferenceMs);
private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
#endif
