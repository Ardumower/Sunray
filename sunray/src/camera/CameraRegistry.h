#pragma once
#ifdef __linux__
#include <string>
#include <vector>

struct CameraInfo {
  std::string path;
  int width{0};
  int height{0};
};

namespace CameraRegistry {
  void refresh();
  const std::vector<CameraInfo>& list();
  // Returns device path for logical index; empty if out of range
  std::string pathForIndex(int idx);
}
#endif

