#ifdef __linux__

#include "LedStripDetector.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include "../../config.h"

// Arduino compatibility headers define these as macros. They break the C++
// standard-library overloads used by the native Linux detector.
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#ifndef DOCK_LED_STRIP_ROI_TOP
#define DOCK_LED_STRIP_ROI_TOP 0.40f
#endif

#ifndef DOCK_LED_STRIP_MIN_CONTRAST
#define DOCK_LED_STRIP_MIN_CONTRAST 0.20f
#endif
#ifndef DOCK_LED_STRIP_MIN_PEAK_DELTA
#define DOCK_LED_STRIP_MIN_PEAK_DELTA 10.0f
#endif
#ifndef DOCK_LED_STRIP_MAX_RELATIVE_COLOR_SPREAD
#define DOCK_LED_STRIP_MAX_RELATIVE_COLOR_SPREAD 0.25f
#endif
#ifndef DOCK_LED_STRIP_MIN_LEDS
#define DOCK_LED_STRIP_MIN_LEDS 5
#endif
#ifndef DOCK_LED_STRIP_MIN_VERTICAL_SPAN
#define DOCK_LED_STRIP_MIN_VERTICAL_SPAN 0.18f
#endif
#ifndef DOCK_LED_STRIP_MAX_SLOPE
#define DOCK_LED_STRIP_MAX_SLOPE 0.12f
#endif
#ifndef DOCK_LED_STRIP_LOST_TIMEOUT_MS
#define DOCK_LED_STRIP_LOST_TIMEOUT_MS 500
#endif
#ifndef DOCK_LED_STRIP_ACQUIRE_FRAMES
#define DOCK_LED_STRIP_ACQUIRE_FRAMES 3
#endif

namespace {

struct BrightSpot {
  float x;
  float y;
  int area;
};

static bool isLedPixel(const uint8_t* pixel) {
  const int lo = std::min((int)pixel[0], std::min((int)pixel[1], (int)pixel[2]));
  const int hi = std::max((int)pixel[0], std::max((int)pixel[1], (int)pixel[2]));
  // Neutral highlights; no absolute exposure-dependent brightness cutoff.
  return hi > 0 && hi - lo <= hi * DOCK_LED_STRIP_MAX_RELATIVE_COLOR_SPREAD;
}

} // namespace

LedStripDetector& LedStripDetector::instance() {
  static LedStripDetector detector;
  return detector;
}

void LedStripDetector::clear() {
  acquireCount_ = 0;
  found_.store(false, std::memory_order_release);
  confidence_.store(0.0f, std::memory_order_relaxed);
  ledCount_.store(0, std::memory_order_relaxed);
  verticalSpan_.store(0.0f, std::memory_order_relaxed);
  timestampMs_.store(0, std::memory_order_relaxed);
}

void LedStripDetector::processRgb(const uint8_t* rgb, int width, int height,
                                  unsigned long timestampMs) {
  if (!rgb || width < 16 || height < 16) {
    clear();
    return;
  }

  // Work on a small image. This keeps the detector inexpensive even when the
  // cloud requests a 1080p camera stream.
  const int workW = std::min(width, 640);
  const int workH = std::max(1, height * workW / width);
  const int pixelCount = workW * workH;
  const int roiTop = std::max(0, std::min(workH - 1, (int)(workH * DOCK_LED_STRIP_ROI_TOP)));
  std::vector<uint8_t> mask((size_t)pixelCount, 0);
  std::vector<uint8_t> visited((size_t)pixelCount, 0);
  std::vector<uint8_t> ridgeMask((size_t)pixelCount, 0);

  // Max-pool neutral luminance so small LEDs survive resizing. Compare it
  // with the local background, not a fixed camera exposure threshold.
  std::vector<float> peak(pixelCount, 0.0f);
  std::vector<float> luminance(pixelCount, 0.0f);
  std::vector<float> integral((workW + 1) * (workH + 1), 0.0f);
  for (int y = 0; y < workH; ++y) {
    float rowSum = 0.0f;
    for (int x = 0; x < workW; ++x) {
      float brightest = 0.0f, mean = 0.0f;
      int samples = 0;
      for (int sy = y * height / workH; sy < (y + 1) * height / workH; ++sy) {
        for (int sx = x * width / workW; sx < (x + 1) * width / workW; ++sx) {
          const uint8_t* px = rgb + ((size_t)sy * width + sx) * 3;
          const float value = std::min((int)px[0], std::min((int)px[1], (int)px[2]));
          mean += value; ++samples;
          if (isLedPixel(px)) brightest = std::max(brightest, value);
        }
      }
      peak[y * workW + x] = brightest;
      luminance[y * workW + x] = mean / std::max(1, samples);
      rowSum += luminance[y * workW + x];
      integral[(y + 1) * (workW + 1) + x + 1] =
          integral[y * (workW + 1) + x + 1] + rowSum;
    }
  }
  const int radius = 10;
  for (int y = 0; y < workH; ++y) {
    for (int x = 0; x < workW; ++x) {
      const int x0 = std::max(0, x - radius), x1 = std::min(workW, x + radius + 1);
      const int y0 = std::max(0, y - radius), y1 = std::min(workH, y + radius + 1);
      const int stride = workW + 1;
      const float background = (integral[y1 * stride + x1] - integral[y0 * stride + x1] -
          integral[y1 * stride + x0] + integral[y0 * stride + x0]) / ((x1 - x0) * (y1 - y0));
      const float value = peak[y * workW + x];
      if (y < roiTop || x < radius || x + radius >= workW) continue;
      const float sideBackground = std::max(luminance[y * workW + x - radius],
                                            luminance[y * workW + x + radius]);
      const bool localContrast = value > background * (1.0f + DOCK_LED_STRIP_MIN_CONTRAST) &&
                                 value - background >= DOCK_LED_STRIP_MIN_PEAK_DELTA;
      const bool ridgeContrast = value > sideBackground * 1.10f &&
                                 value - sideBackground >= DOCK_LED_STRIP_MIN_PEAK_DELTA;
      if (localContrast) mask[y * workW + x] = 1;
      if (localContrast || ridgeContrast) ridgeMask[y * workW + x] = 1;
    }
  }
  float solidSpan = 0.0f, solidX = 0.0f;

  std::vector<BrightSpot> spots;
  std::vector<int> stack;
  stack.reserve(128);
  // Keep separated dim LEDs independent from the wider bright-ridge mask.
  for (int pass = 0; pass < 2; ++pass) {
    if (pass == 1) { mask.swap(ridgeMask); std::fill(visited.begin(), visited.end(), 0); }
  for (int start = 0; start < pixelCount; start++) {
    if (!mask[(size_t)start] || visited[(size_t)start]) continue;
    stack.clear();
    stack.push_back(start);
    visited[(size_t)start] = 1;
    int area = 0;
    int minX = workW, maxX = 0, minY = workH, maxY = 0;
    long sumX = 0, sumY = 0;
    std::vector<int> rowMin(workH, workW), rowMax(workH, -1);
    std::vector<float> rowSum(workH, 0.0f);
    std::vector<int> rowCount(workH, 0);
    while (!stack.empty()) {
      const int pos = stack.back();
      stack.pop_back();
      const int x = pos % workW;
      const int y = pos / workW;
      area++;
      sumX += x;
      sumY += y;
      rowMin[y] = std::min(rowMin[y], x); rowMax[y] = std::max(rowMax[y], x);
      rowSum[y] += x; ++rowCount[y];
      minX = std::min(minX, x); maxX = std::max(maxX, x);
      minY = std::min(minY, y); maxY = std::max(maxY, y);
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) continue;
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= workW || ny < 0 || ny >= workH) continue;
          const int next = ny * workW + nx;
          if (mask[(size_t)next] && !visited[(size_t)next]) {
            visited[(size_t)next] = 1;
            stack.push_back(next);
          }
        }
      }
    }

    const int boxW = maxX - minX + 1;
    const int boxH = maxY - minY + 1;
    // Require a positive horizontal gradient entering the bright ridge and
    // a negative gradient leaving it. A single bright/dark boundary is not
    // a strip. Normalize by ridge brightness to tolerate exposure changes.
    int pairedRows = 0, occupiedRows = 0;
    double fitX = 0, fitY = 0, fitXY = 0, fitYY = 0;
    float ridgeSum = 0.0f;
    for (int y = minY; y <= maxY; ++y) {
      if (!rowCount[y]) continue;
      ++occupiedRows;
      const float centre = rowSum[y] / rowCount[y];
      fitX += centre; fitY += y; fitXY += centre * y; fitYY += y * y;
      const int left = rowMin[y] - 4, right = rowMax[y] + 4;
      if (left < 0 || right >= workW) continue;
      float ridge = 0.0f;
      for (int x = rowMin[y]; x <= rowMax[y]; ++x)
        ridge = std::max(ridge, peak[y * workW + x]);
      ridgeSum += ridge;
      const float rising = ridge - luminance[y * workW + left];
      const float falling = luminance[y * workW + right] - ridge;
      const float minimum = std::max(10.0f, ridge * 0.10f);
      if (rising >= minimum && falling <= -minimum) ++pairedRows;
    }
    const bool pairedEdges = occupiedRows && pairedRows >= occupiedRows * 0.70f;
    const double denominator = occupiedRows * fitYY - fitY * fitY;
    const double slope = denominator > 0 ? (occupiedRows * fitXY - fitX * fitY) / denominator : 0.0;
    const double intercept = occupiedRows ? (fitX - slope * fitY) / occupiedRows : 0.0;
    double residualSquared = 0.0;
    for (int y = minY; y <= maxY; ++y) {
      if (!rowCount[y]) continue;
      const double residual = rowSum[y] / rowCount[y] - (slope * y + intercept);
      residualSquared += residual * residual;
    }
    const bool verticalLine = occupiedRows > 1 && std::fabs(slope) <= DOCK_LED_STRIP_MAX_SLOPE &&
                             residualSquared <= occupiedRows * 1.7 * 1.7;
    // A finite marker has ends. Reject long bright posts continuing beyond
    // the candidate, even when the ROI or local contrast breaks their mask.
    const int endX = std::max(0, std::min(workW - 1, (int)(sumX / area)));
    const bool endsInFrame = minY >= 4 && maxY + 4 < workH;
    const float ridgeMean = occupiedRows ? ridgeSum / occupiedRows : 0.0f;
    const float endBackground = endsInFrame ? std::max(luminance[(minY - 4) * workW + endX],
        luminance[(maxY + 4) * workW + endX]) : 255.0f;
    const bool boundedEnds = endsInFrame && ridgeMean > endBackground * 1.10f &&
                             ridgeMean - endBackground >= 10.0f;
    // At high brightness adjacent LEDs bloom into one continuous strip.
    // Accept only a narrow, tall, filled component; broad bright surfaces
    // and short highlights still fail the geometry checks.
    const float componentSpan = (float)boxH / workH;
    if (boundedEnds && minY > roiTop && pairedEdges && verticalLine && boxW <= std::max(3, workW / 32) && boxH >= boxW * 4 &&
        area >= boxW * boxH * 0.40f &&
        componentSpan >= DOCK_LED_STRIP_MIN_VERTICAL_SPAN && componentSpan > solidSpan) {
      solidSpan = componentSpan;
      solidX = (float)sumX / area;
    }
    // Individual LEDs remain compact. Large white walls, windows and floor
    // reflections are deliberately rejected here.
    if (pass == 0 && pairedEdges && area >= 1 && area <= 120 && boxW <= 16 && boxH <= 16 &&
        boxW <= boxH * 4 && boxH <= boxW * 4) {
      spots.push_back({(float)sumX / area, (float)sumY / area, area});
    }
  }

  } // mask passes

  int bestCount = 0;
  float bestSpan = 0.0f;
  float bestResidual = 1e9f;
  float bestA = 0.0f;
  float bestB = 0.0f;
  float bestMinY = 0.0f;
  float bestMaxY = 0.0f;
  const float tolerance = std::max(1.5f, workW * 0.003f);
  const float minPairSpan = workH * 0.10f;
  const float maxLedGap = std::max(8.0f, workH * 0.025f);

  // Fit a near-vertical line through every plausible spot pair and keep the
  // model supported by the largest vertically distributed LED chain.
  for (size_t i = 0; i < spots.size(); i++) {
    for (size_t j = i + 1; j < spots.size(); j++) {
      const float dy = spots[j].y - spots[i].y;
      if (std::fabs(dy) < minPairSpan) continue;
      const float a = (spots[j].x - spots[i].x) / dy;
      if (std::fabs(a) > DOCK_LED_STRIP_MAX_SLOPE) continue;
      const float b = spots[i].x - a * spots[i].y;
      std::vector<const BrightSpot*> support;
      for (const BrightSpot& spot : spots) {
        const float error = std::fabs(spot.x - (a * spot.y + b));
        if (error <= tolerance) support.push_back(&spot);
      }
      std::sort(support.begin(), support.end(), [](const BrightSpot* lhs, const BrightSpot* rhs) {
        return lhs->y < rhs->y;
      });

      int chainStart = 0;
      for (int end = 0; end < (int)support.size(); end++) {
        if (end > 0 && support[(size_t)end]->y - support[(size_t)end - 1]->y > maxLedGap) {
          chainStart = end;
        }
        const int count = end - chainStart + 1;
        const float minSpotY = support[(size_t)chainStart]->y;
        const float maxSpotY = support[(size_t)end]->y;
        const float span = (maxSpotY - minSpotY) / workH;
        float residual = 0.0f;
        for (int k = chainStart; k <= end; k++) {
          const BrightSpot* spot = support[(size_t)k];
          residual += std::fabs(spot->x - (a * spot->y + b));
        }
        // A strip has repeated LED spacing, unlike arbitrary bright texture.
        float minGap = 1e9f, maxGap = 0.0f;
        for (int k = chainStart + 1; k <= end; ++k) {
          const float gap = support[(size_t)k]->y - support[(size_t)k - 1]->y;
          minGap = std::min(minGap, gap);
          maxGap = std::max(maxGap, gap);
        }
        if (count >= DOCK_LED_STRIP_MIN_LEDS && (minGap < 1.0f || maxGap > minGap * 3.0f)) continue;
        if (count > bestCount ||
            (count == bestCount && span > bestSpan) ||
            (count == bestCount && std::fabs(span - bestSpan) < 0.001f && residual < bestResidual)) {
          bestCount = count;
          bestSpan = span;
          bestResidual = residual;
          bestA = a;
          bestB = b;
          bestMinY = minSpotY;
          bestMaxY = maxSpotY;
        }
      }
    }
  }

  const bool chainFound = bestCount >= DOCK_LED_STRIP_MIN_LEDS &&
                          bestSpan >= DOCK_LED_STRIP_MIN_VERTICAL_SPAN;
  const bool useSolid = !chainFound && solidSpan > 0.0f;
  const bool found = chainFound || useSolid;
#ifdef DOCK_LED_STRIP_DEBUG
  std::fprintf(stderr, "LED model spots=%zu count=%d span=%.3f a=%.3f b=%.3f residual=%.3f\n",
               spots.size(), bestCount, bestSpan, bestA, bestB, bestResidual);
  for (const BrightSpot& spot : spots) {
    const float error = std::fabs(spot.x - (bestA * spot.y + bestB));
    if (error <= tolerance && spot.y >= bestMinY && spot.y <= bestMaxY) {
      std::fprintf(stderr, "  support x=%.2f y=%.2f area=%d error=%.2f\n",
                   spot.x, spot.y, spot.area, error);
    }
  }
#endif
  if (!found) {
    acquireCount_ = 0;
    found_.store(false, std::memory_order_release);
    confidence_.store(0.0f, std::memory_order_relaxed);
    ledCount_.store(bestCount, std::memory_order_relaxed);
    verticalSpan_.store(bestSpan, std::memory_order_relaxed);
    timestampMs_.store(timestampMs, std::memory_order_relaxed);
    return;
  }

  // Aim at the observed chain itself. Extrapolating to the image centre makes
  // a short strip near the image edge unnecessarily sensitive to line slope.
  const float xAtCenter = useSolid ? solidX : bestA * ((bestMinY + bestMaxY) * 0.5f) + bestB;
  if (useSolid) { bestCount = 0; bestSpan = solidSpan; }
  const float horizontalError = std::max(-1.0f, std::min(1.0f,
      (xAtCenter - workW * 0.5f) / (workW * 0.5f)));
  const float countScore = useSolid ? 0.7f : std::min(1.0f, bestCount / 10.0f);
  const float spanScore = std::min(1.0f, bestSpan / 0.45f);
  const float confidence = 0.6f * countScore + 0.4f * spanScore;

  if (acquireCount_ < DOCK_LED_STRIP_ACQUIRE_FRAMES) acquireCount_++;

  horizontalError_.store(horizontalError, std::memory_order_relaxed);
  confidence_.store(confidence, std::memory_order_relaxed);
  ledCount_.store(bestCount, std::memory_order_relaxed);
  verticalSpan_.store(bestSpan, std::memory_order_relaxed);
  timestampMs_.store(timestampMs, std::memory_order_relaxed);
  found_.store(acquireCount_ >= DOCK_LED_STRIP_ACQUIRE_FRAMES, std::memory_order_release);
}

LedStripObservation LedStripDetector::observation(unsigned long nowMs) const {
  LedStripObservation result;
  const unsigned long timestamp = timestampMs_.load(std::memory_order_relaxed);
  result.ageMs = timestamp ? nowMs - timestamp : (unsigned long)-1;
  result.horizontalError = horizontalError_.load(std::memory_order_relaxed);
  result.confidence = confidence_.load(std::memory_order_relaxed);
  result.ledCount = ledCount_.load(std::memory_order_relaxed);
  result.verticalSpan = verticalSpan_.load(std::memory_order_relaxed);
  result.found = found_.load(std::memory_order_acquire) &&
                 timestamp && result.ageMs <= DOCK_LED_STRIP_LOST_TIMEOUT_MS;
  return result;
}

#endif
