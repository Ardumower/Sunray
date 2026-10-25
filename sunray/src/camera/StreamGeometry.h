#pragma once
#include <algorithm>
#include <cstdint>

namespace CameraStream {
struct Size { int width, height; };

// Requested dimensions are a bounding box. Preserve the whole source image
// instead of stretching a widescreen camera into a 4:3 preview.
inline Size fit(int sourceWidth, int sourceHeight, int maxWidth, int maxHeight) {
  if (sourceWidth <= 0 || sourceHeight <= 0 || maxWidth <= 0 || maxHeight <= 0)
    return {1, 1};
  if (int64_t(sourceWidth) * maxHeight > int64_t(sourceHeight) * maxWidth)
    return {maxWidth, std::max(1, int(int64_t(sourceHeight) * maxWidth / sourceWidth))};
  return {std::max(1, int(int64_t(sourceWidth) * maxHeight / sourceHeight)), maxHeight};
}
}
