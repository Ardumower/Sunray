#pragma once
#ifdef __linux__
#include <cstdint>
#include <memory>
#include <vector>
// Camera-only visualization. No connection to motion/navigation decisions.
// One pending frame and one completed frame; inference never blocks capture.
class GrassOverlay {
public:
  GrassOverlay();
  ~GrassOverlay();
  void submit(const uint8_t* rgb, int width, int height, uint64_t timestamp);
  bool latest(std::vector<uint8_t>& rgb, uint64_t& timestamp, double& inferenceMs);
private:
  struct Impl;
  std::unique_ptr<Impl> impl;
};
#endif
