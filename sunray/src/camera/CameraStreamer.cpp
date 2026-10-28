#ifdef __linux__
#include "CameraStreamer.h"
#include "GrassOverlay.h"
#include "TextureFiles.h"
#include "GrassObstacle.h"
#include "NavigationOverlay.h"
#include "MediaStream.h"
#include "CameraRegistry.h"
#include "ReflectorStripsDetector.h"
#include "SafeJpeg.h"
#include "ReflectorFailureRecorder.h"
#include "CameraLightLogger.h"
#include "ImageDarkness.h"
#include "LatestFrameWorker.h"
#include "PacedFrameWorker.h"
#include "FrameTimestamp.h"
#include "StreamGeometry.h"
#include "Console.h"
#include <Sim3D.h>
// Access motor speed setpoints via C-accessors (to avoid Arduino header conflicts)
extern "C" float cameraGetLinearSet();
extern "C" float cameraGetAngularSet();
// Provide weak fallbacks (return 0) in case OverlayData.cpp is not linked
extern "C" __attribute__((weak)) float cameraGetLinearSet() { return 0.0f; }
extern "C" __attribute__((weak)) float cameraGetAngularSet() { return 0.0f; }
extern "C" __attribute__((weak)) int cameraGetNavigationMode() { return NavigationOverlay::Hidden; }
// Default to LinuxConsole if CONSOLE macro is not mapped by a config
#ifndef CONSOLE
#define CONSOLE Console
#endif
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include <ctime>

// Minimal 1x1 JPEG placeholder
static const unsigned char kTinyJpeg[] = {
  0xFF,0xD8,0xFF,0xE0,0x00,0x10,0x4A,0x46,0x49,0x46,0x00,0x01,0x01,0x01,0x00,0x60,
  0x00,0x60,0x00,0x00,0xFF,0xDB,0x00,0x43,0x00,0x08,0x06,0x06,0x07,0x06,0x05,0x08,
  0x07,0x07,0x07,0x09,0x09,0x08,0x0A,0x0C,0x14,0x0D,0x0C,0x0B,0x0B,0x0C,0x19,0x12,
  0x13,0x0F,0x14,0x1D,0x1A,0x1F,0x1E,0x1D,0x1A,0x1C,0x1C,0x20,0x24,0x2E,0x27,0x20,
  0x22,0x2C,0x23,0x1C,0x1C,0x28,0x37,0x29,0x2C,0x30,0x31,0x34,0x34,0x34,0x1F,0x27,
  0x39,0x3D,0x38,0x32,0x3C,0x2E,0x33,0x34,0x32,0xFF,0xDB,0x00,0x43,0x01,0x09,0x09,
  0x09,0x0C,0x0B,0x0C,0x18,0x0D,0x0D,0x18,0x32,0x21,0x1C,0x21,0x32,0x32,0x32,0x32,
  0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,
  0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,
  0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0x32,0xFF,0xC0,0x00,0x11,0x08,0x00,0x01,
  0x00,0x01,0x03,0x01,0x22,0x00,0x02,0x11,0x01,0x03,0x11,0x01,0xFF,0xC4,0x00,0x1F,
  0x00,0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0xFF,0xC4,0x00,
  0xB5,0x10,0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,
  0x01,0x7D,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,
  0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,
  0xD1,0xF0,0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,
  0x27,0x28,0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,
  0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,
  0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,
  0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,
  0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,
  0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,
  0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,
  0xF7,0xF8,0xF9,0xFA,0xFF,0xDA,0x00,0x0C,0x03,0x01,0x00,0x02,0x11,0x03,0x11,0x00,
  0x3F,0x00,0xFF,0xD9
};

CameraStreamer& CameraStreamer::instance() { static CameraStreamer inst; return inst; }
CameraStreamer::CameraStreamer() {}
CameraStreamer::~CameraStreamer() {
  streamingRequested_.store(false);
  reflectorDetectionRequested_.store(false);
  grassMonitoringRequested_.store(false);
  running_.store(false);
  try { if (worker_.joinable()) worker_.join(); } catch (...) {}
}

void CameraStreamer::setSender(Sender s) { std::lock_guard<std::mutex> lk(mtx_); sender_ = std::move(s); }

void CameraStreamer::start(int index, int width, int height, int fps, int quality, bool auxiliaryViews) {
  camIndex_.store(index);
  reqW_.store(width);
  reqH_.store(height);
  reqFps_.store((fps <= 0) ? 5 : fps);
  if (quality < 10) quality = 10; if (quality > 95) quality = 95;
  reqQ_.store(quality);
  auxiliaryViews_.store(auxiliaryViews);
  streamingRequested_.store(true);
  CONSOLE.print("CAM start idx="); CONSOLE.print(index);
  CONSOLE.print(" req="); CONSOLE.print(width); CONSOLE.print("x"); CONSOLE.print(height);
  CONSOLE.print(" @"); CONSOLE.println((int)fps);
  ensureWorkerRunning();
}

void CameraStreamer::stop() {
  streamingRequested_.store(false);
  CONSOLE.println("CAM stream stop");
  stopWorkerIfUnused();
}

void CameraStreamer::enableReflectorStripsDetection(int index, int fps) {
  reflectorCamIndex_.store(index);
  reflectorFps_.store((fps <= 0) ? 10 : fps);
  reflectorDetectionRequested_.store(true);
  CONSOLE.print("CAM reflectors detector start idx="); CONSOLE.print(index);
  CONSOLE.print(" @"); CONSOLE.println((fps <= 0) ? 10 : fps);
  ensureWorkerRunning();
}

void CameraStreamer::disableReflectorStripsDetection() {
  reflectorDetectionRequested_.store(false);
  ReflectorStripsDetector::instance().clear();
  CONSOLE.println("CAM reflectors detector stop");
  stopWorkerIfUnused();
}

int CameraStreamer::captureIndex() const {
  if(reflectorDetectionRequested_.load())return reflectorCamIndex_.load();
  if(grassMonitoringRequested_.load() || (camIndex_.load()==100 || camIndex_.load()==101))return 0;
  return camIndex_.load();
}
void CameraStreamer::setGrassMonitoring(bool enabled) {
  if(grassMonitoringRequested_.exchange(enabled)==enabled)return;
  if(enabled)ensureWorkerRunning();else stopWorkerIfUnused();
}
extern "C" void cameraSetGrassMonitoring(bool enabled){CameraStreamer::instance().setGrassMonitoring(enabled);}
void CameraStreamer::ensureWorkerRunning() {
  if (running_.exchange(true)) return;
  try { if (worker_.joinable()) worker_.join(); } catch (...) {}
  worker_ = std::thread([this]() { runLoop(); });
}

void CameraStreamer::stopWorkerIfUnused() {
  if (streamingRequested_.load() || reflectorDetectionRequested_.load() || grassMonitoringRequested_.load()) return;
  running_.store(false);
  try { if (worker_.joinable()) worker_.join(); } catch (...) {}
  CONSOLE.println("CAM capture stop");
}

struct MMapBuffer { void* start{nullptr}; size_t length{0}; };

// --- Simple OSD drawing helpers (RGB888) ---
static inline void putPixel(uint8_t* img, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
  size_t idx = ((size_t)y * (size_t)w + (size_t)x) * 3;
  img[idx + 0] = r; img[idx + 1] = g; img[idx + 2] = b;
}

static void fillRect(uint8_t* img, int w, int h, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
  if (x0 > x1) std::swap(x0, x1); if (y0 > y1) std::swap(y0, y1);
  if (x0 >= w || y0 >= h || x1 < 0 || y1 < 0) return;
  x0 = std::max(0, x0); y0 = std::max(0, y0); x1 = std::min(w - 1, x1); y1 = std::min(h - 1, y1);
  for (int y = y0; y <= y1; y++) {
    for (int x = x0; x <= x1; x++) {
      if (a == 255) { putPixel(img, w, h, x, y, r, g, b); }
      else {
        // simple alpha blend over existing pixel
        size_t idx = ((size_t)y * (size_t)w + (size_t)x) * 3;
        uint8_t br = img[idx+0], bg = img[idx+1], bb = img[idx+2];
        img[idx+0] = (uint8_t)((r * a + br * (255 - a)) / 255);
        img[idx+1] = (uint8_t)((g * a + bg * (255 - a)) / 255);
        img[idx+2] = (uint8_t)((b * a + bb * (255 - a)) / 255);
      }
    }
  }
}

// 5x7 pixel font for digits, stored top row first (bit 6).
static const uint8_t kFont5x7[][5] = {
  // '0'..'9'
  {0x3E,0x45,0x49,0x51,0x3E}, // 0
  {0x00,0x21,0x7F,0x01,0x00}, // 1
  {0x23,0x45,0x49,0x49,0x31}, // 2
  {0x22,0x41,0x49,0x49,0x36}, // 3
  {0x0C,0x14,0x24,0x7F,0x04}, // 4
  {0x72,0x51,0x51,0x51,0x4E}, // 5
  {0x1E,0x29,0x49,0x49,0x06}, // 6
  {0x40,0x47,0x48,0x50,0x60}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x30,0x49,0x49,0x4A,0x3C}, // 9
};

static const uint8_t kLetters5x7[][5] = {
  {0x3f,0x48,0x48,0x48,0x3f}, // A
  {0x0,0x0,0x0,0x0,0x0}, // B
  {0x3e,0x41,0x41,0x41,0x22}, // C
  {0x7f,0x41,0x41,0x41,0x3e}, // D
  {0x7f,0x49,0x49,0x49,0x41}, // E
  {0x7f,0x48,0x48,0x48,0x40}, // F
  {0x3e,0x41,0x49,0x49,0x2e}, // G
  {0x7f,0x8,0x8,0x8,0x7f}, // H
  {0x41,0x41,0x7f,0x41,0x41}, // I
  {0x0,0x0,0x0,0x0,0x0}, // J
  {0x7f,0x8,0x14,0x22,0x41}, // K
  {0x7f,0x1,0x1,0x1,0x1}, // L
  {0x7f,0x20,0x18,0x20,0x7f}, // M
  {0x0,0x0,0x0,0x0,0x0}, // N
  {0x3e,0x41,0x41,0x41,0x3e}, // O
  {0x7f,0x48,0x48,0x48,0x30}, // P
  {0x0,0x0,0x0,0x0,0x0}, // Q
  {0x7f,0x48,0x4c,0x4a,0x31}, // R
  {0x31,0x49,0x49,0x49,0x46}, // S
  {0x40,0x40,0x7f,0x40,0x40}, // T
  {0x7e,0x1,0x1,0x1,0x7e}, // U
  {0x0,0x0,0x0,0x0,0x0}, // V
  {0x7e,0x1,0xe,0x1,0x7e}, // W
  {0x63,0x14,0x8,0x14,0x63}, // X
  {0x0,0x0,0x0,0x0,0x0}, // Y
  {0x0,0x0,0x0,0x0,0x0}, // Z
};

static void drawChar5x7(uint8_t* img, int w, int h, int x, int y, char c, int scale, uint8_t r, uint8_t g, uint8_t b) {
  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z')) {
    const uint8_t* col = c >= 'A' ? kLetters5x7[c - 'A'] : kFont5x7[c - '0'];
    for (int cx = 0; cx < 5; cx++) {
      uint8_t bits = col[cx];
      for (int cy = 0; cy < 7; cy++) {
        bool on = (bits >> (7 - 1 - cy)) & 1; // top bit is row 0
        if (on) {
          for (int yy = 0; yy < scale; yy++)
            for (int xx = 0; xx < scale; xx++)
              putPixel(img, w, h, x + cx*scale + xx, y + cy*scale + yy, r, g, b);
        }
      }
    }
  } else if (c == ':') {
    // draw two dots
    int dotSize = scale;
    fillRect(img, w, h, x + 2*scale, y + 2*scale, x + 2*scale + dotSize - 1, y + 2*scale + dotSize - 1, r, g, b);
    fillRect(img, w, h, x + 2*scale, y + 5*scale, x + 2*scale + dotSize - 1, y + 5*scale + dotSize - 1, r, g, b);
  } else if (c == '.') {
    fillRect(img, w, h, x + 2*scale, y + 6*scale,
             x + 3*scale - 1, y + 7*scale - 1, r, g, b);
  }
}

#else // !__linux__

// When not building for Linux, provide no-op C hooks so the
// rest of the firmware can link without pulling in Linux-only camera code.
extern "C" void cameraStreamerStart(int, int, int, int, int, bool) {}
extern "C" void cameraStreamerStop() {}
extern "C" void cameraReflectorStripsEnable(int, int) {}
extern "C" bool cameraReflectorStripsFound() { return false; }
extern "C" float cameraReflectorStripsHorizontalError() { return 0.0f; }
extern "C" float cameraReflectorStripsHorizontalSpan() { return 0.0f; }
extern "C" float cameraReflectorStripsConfidence() { return 0.0f; }
extern "C" int cameraReflectorStripsStripCount() { return 0; }
extern "C" unsigned long cameraReflectorStripsAgeMs() { return (unsigned long)-1; }

#endif // __linux__

#ifdef __linux__

static void drawText(uint8_t* img, int w, int h, int x, int y, const std::string& text, int scale, uint8_t r, uint8_t g, uint8_t b) {
  int cx = x;
  for (char c : text) {
    drawChar5x7(img, w, h, cx, y, c, scale, r, g, b);
    cx += (5 + 1) * scale; // 1px space
  }
}

static void drawFrameTimestamp(uint8_t* img, int w, int h, const std::string& label) {
  const int scale = std::max(1, w / 320);
  const int pad = 3 * scale;
  const int textWidth = int(label.size()) * 6 * scale;
  const int x = std::max(pad, w - textWidth - pad);
  const int y = std::max(pad, h - 7 * scale - 2 * pad);
  fillRect(img, w, h, x - pad, y - pad, w - 1, h - 1, 0, 0, 0, 220);
  drawText(img, w, h, x, y, label, scale, 255, 255, 255);
}

static void drawVerticalArrow(uint8_t* img, int w, int h, int cx, int cy, int len, int thickness, bool up, uint8_t r, uint8_t g, uint8_t b) {
  if (len <= 0) return;
  int y0 = cy;
  int y1 = up ? (cy - len) : (cy + len);
  // shaft
  fillRect(img, w, h, cx - thickness/2, std::min(y0, y1), cx + (thickness-1)/2, std::max(y0, y1), r, g, b);
  // head: small triangle
  int hy = y1;
  int dir = up ? -1 : 1;
  // Flip head to extend opposite to shaft direction so the tip is at y1
  int headDir = -dir;
  for (int i = 0; i < thickness*2 + 6; i++) {
    int half = i/2;
    fillRect(img, w, h, cx - half, hy + headDir*i, cx + half, hy + headDir*i, r, g, b);
  }
}

static void drawHorizontalArrow(uint8_t* img, int w, int h, int cx, int cy, int len, int thickness, bool right, uint8_t r, uint8_t g, uint8_t b) {
  if (len <= 0) return;
  int x0 = cx;
  int x1 = right ? (cx + len) : (cx - len);
  // shaft
  fillRect(img, w, h, std::min(x0, x1), cy - thickness/2, std::max(x0, x1), cy + (thickness-1)/2, r, g, b);
  // head
  int hx = x1;
  int dir = right ? 1 : -1;
  // Flip head to extend opposite to shaft direction so the tip is at x1
  int headDir = -dir;
  for (int i = 0; i < thickness*2 + 6; i++) {
    int half = i/2;
    fillRect(img, w, h, hx + headDir*i, cy - half, hx + headDir*i, cy + half, r, g, b);
  }
}

static void drawOverlay(uint8_t* img, int w, int h) {
  // Joystick arrows from last AT+M (linear m/s, angular rad/s)
  float lin = cameraGetLinearSet();   // positive => forward
  float ang = cameraGetAngularSet();  // positive => rotate right
  const float maxLin = 0.7f;   // m/s typical max
  const float maxAng = 1.0f;   // rad/s typical max
  int r = (int)(std::min(w, h) * 0.35f);                     // joystick circle radius approx
  // keep vertical arrow length behavior as before (worked well)
  int vLen = (int)(std::min(1.0f, std::fabs(lin) / maxLin) * (h / 3));
  int hLen = (int)(std::min(1.0f, std::fabs(ang) / maxAng) * r);
  int thick = std::max(3, std::min(w, h) / 120);

  // positions: restore previous vertical arrow position, and move horizontal arrow near bottom of right circle
  int pad = std::max(2, w/160);
  int scale = std::max(2, w/160);
  int vCx = pad + 6*scale; 
  int vCy = h / 2;
  int hCx = (int)(w * 0.75f);
  // Place horizontal arrow further below top overlays (~35% from top)
  int hCy = std::max(pad + 6*scale, (int)(h * 0.35f));

  // arrows: keep original shaft directions; only heads are flipped in draw* functions
  if (vLen > 0) drawVerticalArrow(img, w, h, vCx, vCy, vLen, thick, /*up=*/(lin >= 0), 0, 255, 0);
  // Positive angular means turn left (CCW): show left arrow for ang > 0
  if (hLen > 0) drawHorizontalArrow(img, w, h, hCx, hCy, hLen, thick, /*right=*/(ang < 0), 0, 200, 255);
}

static void drawNavigationOverlay(uint8_t* img, int w, int h) {
  const auto mode = static_cast<NavigationOverlay::Mode>(cameraGetNavigationMode());
  const char* label = NavigationOverlay::label(mode);
  if (!*label) return;
  const int scale = std::max(1, w / 160);
  const int pad = 3 * scale;
  const int width = (int)std::strlen(label) * 6 * scale + 2 * pad;
  fillRect(img, w, h, pad, pad, pad + width, pad + 7 * scale + 2 * pad, 0, 0, 0, 210);
  const bool waiting = mode == NavigationOverlay::CameraWait || mode == NavigationOverlay::CameraSearch ||
      mode == NavigationOverlay::CameraStop || mode == NavigationOverlay::GpsWait;
  drawText(img, w, h, 2 * pad, 2 * pad, label, scale,
           waiting ? 255 : 100, waiting ? 210 : 255, waiting ? 70 : 255);
}

static void drawDarknessOverlay(uint8_t* img,int w,int h,bool dark) {
  if(!dark)return;
  const int scale=std::max(1,w/160),pad=3*scale;
  const int y=h-7*scale-3*pad;
  fillRect(img,w,h,pad,y-pad,pad+8*6*scale+2*pad,h-pad,75,0,0,230);
  drawText(img,w,h,2*pad,y,"TOO DARK",scale,255,215,80);
}

static bool jpegDecodeToRGB(const uint8_t* data, size_t len, std::vector<uint8_t>& outRGB, int& w, int& h) {
  return SafeJpeg::decode(data, len, outRGB, w, h);
}

static void resizeNearestRGB(const uint8_t* src, int sw, int sh, uint8_t* dst, int dw, int dh) {
  for (int y = 0; y < dh; y++) {
    int sy = (y * sh) / dh; const uint8_t* sp = src + (size_t)sy * (size_t)sw * 3; uint8_t* dp = dst + (size_t)y * (size_t)dw * 3;
    for (int x = 0; x < dw; x++) { int sx = (x * sw) / dw; const uint8_t* s = sp + (size_t)sx * 3; dp[x*3+0] = s[0]; dp[x*3+1] = s[1]; dp[x*3+2] = s[2]; }
  }
}

static bool jpegEncodeRGB(const uint8_t* rgb, int w, int h, int quality, std::vector<uint8_t>& outJpeg) {
  jpeg_compress_struct cinfo; jpeg_error_mgr jerr; cinfo.err = jpeg_std_error(&jerr); jpeg_create_compress(&cinfo);
  unsigned char* outBuf = nullptr; unsigned long outSize = 0; jpeg_mem_dest(&cinfo, &outBuf, &outSize);
  cinfo.image_width = w; cinfo.image_height = h; cinfo.input_components = 3; cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo); jpeg_set_quality(&cinfo, quality, TRUE); jpeg_start_compress(&cinfo, TRUE);
  while (cinfo.next_scanline < cinfo.image_height) { JSAMPROW rowptr = (JSAMPROW)&rgb[(size_t)cinfo.next_scanline * (size_t)w * 3]; jpeg_write_scanlines(&cinfo, &rowptr, 1); }
  jpeg_finish_compress(&cinfo); outJpeg.assign(outBuf, outBuf + outSize); jpeg_destroy_compress(&cinfo); free(outBuf); return true;
}

void CameraStreamer::runLoop() {
  using namespace std::chrono;
  while(running_.load()) {
    const int index=captureIndex();
    runCaptureSession();
    // The session has joined its detector before returning. No pending frame
    // can republish a marker after this clear, including after USB removal.
    ReflectorStripsDetector::instance().clear();
    const auto retryAt=steady_clock::now()+seconds(2);
    auto nextFrame=steady_clock::now();
    while(running_.load() && captureIndex()==index && steady_clock::now()<retryAt) {
      const auto now=steady_clock::now();
      if(streamingRequested_.load() && now>=nextFrame) {
        buildAndSendFrame();
        nextFrame=now+milliseconds(std::max(1,1000/std::max(1,reqFps_.load())));
      }
      std::this_thread::sleep_for(milliseconds(50));
    }
  }
}

void CameraStreamer::runCaptureSession() {
  using namespace std::chrono;
  const CameraFrameClock frameClock;
  auto videoStatsAt=steady_clock::now(),previousVideoSend=videoStatsAt;
  unsigned videoFrames=0;int statsIndex=-1;
  double maxVideoGapMs=0,maxCaptureAgeMs=0,maxEncodeMs=0,maxHandoffMs=0;
  // Open V4L2 device
  int index = captureIndex();
  int outW = reqW_.load(); int outH = reqH_.load(); int fps = reqFps_.load();
  const bool virtualCamera=Sim3D::enabled();
  uint64_t virtualSequence=0;
  int fd=-1;
  struct CaptureResources {
    int fd; bool streaming=false;
    std::vector<MMapBuffer> buffers;
    explicit CaptureResources(int value):fd(value){}
    ~CaptureResources(){
      if(streaming){int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(fd,VIDIOC_STREAMOFF,&type);}
      for(auto& b:buffers)if(b.start && b.length)munmap(b.start,b.length);
      ::close(fd);
    }
  } capture(-1);
  auto& bufs=capture.buffers;
  v4l2_requestbuffers req{};
  if(!virtualCamera){
  CameraRegistry::refresh();
  std::string devPathStr = CameraRegistry::pathForIndex(index);
  char devPath[64];
  if (devPathStr.empty()) {
    CONSOLE.print("CAM index invalid: "); CONSOLE.println(index);
    return;
  }
  std::snprintf(devPath, sizeof(devPath), "%s", devPathStr.c_str());
  fd = ::open(devPath, O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    CONSOLE.print("CAM open failed: "); CONSOLE.println(devPath);
    return;
  }

  capture.fd=fd;

  CONSOLE.print("CAM opened "); CONSOLE.println(devPath);
  // Try to set format to MJPEG
  v4l2_format fmt; memset(&fmt, 0, sizeof(fmt)); fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(fd,VIDIOC_G_FMT,&fmt)<0){CONSOLE.println("CAM get format failed");return;}
  fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
  if(ioctl(fd,VIDIOC_S_FMT,&fmt)<0 || ioctl(fd,VIDIOC_G_FMT,&fmt)<0){
    CONSOLE.println("CAM set format failed");return;
  }
  CONSOLE.print("CAM source "); CONSOLE.print((int)fmt.fmt.pix.width); CONSOLE.print("x"); CONSOLE.println((int)fmt.fmt.pix.height);
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG || (int)fmt.fmt.pix.width <= 0 || (int)fmt.fmt.pix.height <= 0) {
    CONSOLE.println("CAM invalid source format, fallback red frames");
    return;
  }
  // Request MMAP
  memset(&req, 0, sizeof(req)); req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
    CONSOLE.println("CAM reqbufs failed");
    return;
  }
  capture.buffers.resize(req.count);
  for (unsigned i = 0; i < req.count; i++) {
    v4l2_buffer b; memset(&b, 0, sizeof(b)); b.type = req.type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { CONSOLE.println("CAM querybuf failed"); return; }
    void* start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (start == MAP_FAILED) { CONSOLE.println("CAM mmap failed"); return; }
    bufs[i].start = start; bufs[i].length = b.length;
  }
  for (unsigned i = 0; i < req.count; i++) { v4l2_buffer b; memset(&b, 0, sizeof(b)); b.type = req.type; b.memory = V4L2_MEMORY_MMAP; b.index = i; if(ioctl(fd, VIDIOC_QBUF, &b)<0){CONSOLE.println("CAM initial qbuf failed");return;} }
  int type = req.type;
  if(ioctl(fd,VIDIOC_STREAMON,&type)<0){CONSOLE.println("CAM streamon failed");return;}
  capture.streaming=true;
  } else { CONSOLE.println("CAM source: Godot 3D simulator"); }

  auto lastDetection = steady_clock::now() - milliseconds(1000);
  bool reflectorStatusInitialized = false;
  bool lastReflectorFound = false;
  ReflectorFailureRecorder failureRecorder;
  CameraLightLogger lightLogger;
  struct ReflectorFrame {
    std::vector<uint8_t> rgb,jpeg;
    int width=0,height=0;
    unsigned long timestampMs=0;
    float linear=0,angular=0;
  };
  std::atomic<unsigned> detectionsFound{0},detectionsMissing{0};
  auto lastDetectorTiming=steady_clock::now();
  unsigned long maxDetectorProcessMs=0,maxDetectorQueueMs=0;

  LatestFrameWorker<ReflectorFrame> reflectorWorker([&](const ReflectorFrame& frame){
    if(!reflectorDetectionRequested_.load())return;
    const auto nowMs=duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    // Never refresh navigation with a queued image that has already expired.
    if(nowMs-frame.timestampMs>500)return;
    const auto processStart=steady_clock::now();
    ReflectorStripsDetector::instance().processRgb(frame.rgb.data(), frame.width, frame.height, frame.timestampMs);
    const auto processEnd=steady_clock::now();
    maxDetectorQueueMs=std::max(maxDetectorQueueMs,(unsigned long)(nowMs-frame.timestampMs));
    maxDetectorProcessMs=std::max(maxDetectorProcessMs,(unsigned long)duration_cast<milliseconds>(processEnd-processStart).count());
    if(processEnd-lastDetectorTiming>=seconds(5)){
      CONSOLE.print("CAM reflectors timing maxProcessMs=");CONSOLE.print(maxDetectorProcessMs);
      CONSOLE.print(" maxQueueMs=");CONSOLE.println(maxDetectorQueueMs);
      maxDetectorProcessMs=maxDetectorQueueMs=0;lastDetectorTiming=processEnd;
    }
    const ReflectorStripsObservation marker = ReflectorStripsDetector::instance().observation(frame.timestampMs);
    if(marker.found)++detectionsFound;else ++detectionsMissing;
    failureRecorder.observe(frame.jpeg.data(), frame.jpeg.size(), frame.width, frame.height, frame.timestampMs, marker, frame.linear, frame.angular);
    if (!reflectorStatusInitialized || marker.found != lastReflectorFound) {
      reflectorStatusInitialized = true;
      lastReflectorFound = marker.found;
      CONSOLE.print("CAM reflectors found="); CONSOLE.print(marker.found);
      CONSOLE.print(" strips="); CONSOLE.print(marker.stripCount);
      CONSOLE.print(" mode="); CONSOLE.print(marker.found ? "triple" : "none");
      CONSOLE.print(" confidence="); CONSOLE.print(marker.confidence);
      CONSOLE.print(" error="); CONSOLE.println(marker.horizontalError);
    }
  });
  auto lastFreshnessReport=steady_clock::now();
  unsigned long maximumMarkerAge=0;
  unsigned markerSamples=0,staleMarkerSamples=0;
  ImageDarkness darkness;
  GrassOverlay grass;
  auto lastGrass=steady_clock::now()-seconds(1);
  auto lastTexture=steady_clock::now()-seconds(1);
  uint64_t lastTextureView=0;
  auto lastLightMeasurement=steady_clock::now()-seconds(1);
  auto lastBadFrameLog = steady_clock::now() - seconds(5);
  struct VideoFrame {
    std::vector<uint8_t> rgb;
    int width=0,height=0;
    unsigned long timestampMs=0;
    bool dark=false;
  };
  PacedFrameWorker<VideoFrame> videoWorker(fps,[&](const VideoFrame& frame){
    if(!streamingRequested_.load())return;
    const auto timestampMs=frame.timestampMs;
    const auto age=duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()-timestampMs;
    if(age>500)return; // Do not turn a stalled capture into apparently live video.
    const auto& rgb=frame.rgb;
    const int srcW=frame.width,srcH=frame.height;
    int outW=reqW_.load(),outH=reqH_.load();
    const int outputIndex=camIndex_.load();
    const auto size=CameraStream::fit(srcW,srcH,outW,outH);
    outW=size.width; outH=size.height;
    std::vector<uint8_t> outRGB((size_t)outW * (size_t)outH * 3);
    resizeNearestRGB(rgb.data(), srcW, srcH, outRGB.data(), outW, outH);
    std::vector<uint8_t> rawView;
    if(outputIndex==101)rawView=outRGB;
    uint64_t frameTimestamp = timestampMs;
    if (outputIndex == 100) {
      std::vector<uint8_t> classified; uint64_t stamp=0; double ms=0;
      if (grass.latest(classified, stamp, ms) && timestampMs >= stamp && timestampMs-stamp < 1000) {
        resizeNearestRGB(classified.data(),640,360,outRGB.data(),outW,outH);
        frameTimestamp = stamp;
        char label[48]; snprintf(label,sizeof(label),"GRASS %.0f MS",ms);
        drawText(outRGB.data(),outW,outH,4,20,label,1,255,255,255);
      } else {
        // Clearly mark missing/stale output, never present it as a classification.
        drawText(outRGB.data(),outW,outH,4,20,"GRASS WAIT",1,255,180,0);
      }
    }
    if (outputIndex == 101) {
      std::vector<uint8_t> jpeg, classified; uint64_t stamp=0; int w=0,h=0;
      if(TextureFiles::overlay(jpeg,timestampMs,stamp) && jpegDecodeToRGB(jpeg.data(),jpeg.size(),classified,w,h)){
        resizeNearestRGB(classified.data(),w,h,outRGB.data(),outW,outH);frameTimestamp=stamp;
      }else drawText(outRGB.data(),outW,outH,4,20,"TRAIN WAIT - NO RESULT",1,255,180,0);
    }
    if (outputIndex != 100 && outputIndex != 101) {
      GrassObstacle::drawContour(outRGB.data(),outW,outH,GrassOverlay::obstacleState());
    }
    auto overlayView = [&](std::vector<uint8_t>& pixels) {
    // Overlay: time and joystick arrows
    drawOverlay(pixels.data(), outW, outH);
    if (reflectorDetectionRequested_.load()) {
      const ReflectorStripsObservation marker = ReflectorStripsDetector::instance().observation(timestampMs);
      const int centreX = outW / 2;
      fillRect(pixels.data(), outW, outH, centreX - 1, 0, centreX + 1, outH - 1,
               0, 120, 255, 150);
      if (marker.found) {
        const int reflectorX = (int)((marker.horizontalError * 0.5f + 0.5f) * outW);
        fillRect(pixels.data(), outW, outH, reflectorX - 2, 0, reflectorX + 2, outH - 1,
                 0, 255, 0, 190);
      }
    }
    drawNavigationOverlay(pixels.data(), outW, outH);
    drawDarknessOverlay(pixels.data(), outW, outH,frame.dark);
    };
    Sender sender; { std::lock_guard<std::mutex> lk(mtx_); sender=sender_; }
    const int q=reqQ_.load();
    auto sendView = [&](std::vector<uint8_t>& pixels,uint64_t stamp,int view) {
      if(!sender)return;
      overlayView(pixels);
      drawFrameTimestamp(pixels.data(),outW,outH,frameClock.label(stamp));
      std::vector<uint8_t> jpeg;
      const auto encodeStarted=steady_clock::now();
      if(!jpegEncodeRGB(pixels.data(),outW,outH,q>0?q:70,jpeg))return;
      const double encodeMs=duration<double,std::milli>(steady_clock::now()-encodeStarted).count();
      uint8_t hdr[16];
      buildMediaHeader(hdr,1,(uint16_t)outW,(uint16_t)outH,uint32_t(stamp),uint16_t(0x8000|(view&0xff)));
      std::vector<uint8_t> packet(sizeof(hdr)+jpeg.size());
      std::memcpy(packet.data(),hdr,sizeof(hdr));
      std::memcpy(packet.data()+sizeof(hdr),jpeg.data(),jpeg.size());
      const auto handedAt=steady_clock::now();
      sender(packet.data(),packet.size());
      const double handoffMs=duration<double,std::milli>(steady_clock::now()-handedAt).count();
      if(view==outputIndex){
        if(statsIndex!=view){statsIndex=view;videoFrames=0;videoStatsAt=handedAt;maxVideoGapMs=maxCaptureAgeMs=maxEncodeMs=maxHandoffMs=0;}
        if(videoFrames)maxVideoGapMs=std::max(maxVideoGapMs,duration<double,std::milli>(handedAt-previousVideoSend).count());
        previousVideoSend=handedAt;++videoFrames;
        const auto handedMs=duration_cast<milliseconds>(handedAt.time_since_epoch()).count();
        if(uint64_t(handedMs)>=stamp)maxCaptureAgeMs=std::max(maxCaptureAgeMs,double(handedMs-stamp));
        maxEncodeMs=std::max(maxEncodeMs,encodeMs);maxHandoffMs=std::max(maxHandoffMs,handoffMs);
        const double seconds=duration<double>(handedAt-videoStatsAt).count();
        if(seconds>=10){
          std::fprintf(stderr,"CAM pipeline idx=%d frames=%u fps=%.2f gapMaxMs=%.1f captureAgeMaxMs=%.1f encodeMaxMs=%.1f handoffMaxMs=%.1f\n",view,videoFrames,videoFrames/seconds,maxVideoGapMs,maxCaptureAgeMs,maxEncodeMs,maxHandoffMs);
          videoStatsAt=handedAt;videoFrames=0;maxVideoGapMs=maxCaptureAgeMs=maxEncodeMs=maxHandoffMs=0;
        }
      }
    };
    sendView(outRGB,frameTimestamp,outputIndex);
    // Normal and texture views share one physical capture. Tag and send
    // them separately so concurrent browsers cannot receive each other's
    // overlay. An auxiliary texture frame is sent only when it is new.
    if(auxiliaryViews_.load() && outputIndex==101){
      GrassObstacle::drawContour(rawView.data(),outW,outH,GrassOverlay::obstacleState());
      sendView(rawView,timestampMs,0);
    }else if(auxiliaryViews_.load() && outputIndex==0){
      std::vector<uint8_t> jpeg,classified;uint64_t stamp=0;int w=0,h=0;
      if(TextureFiles::overlay(jpeg,timestampMs,stamp) && stamp!=lastTextureView &&
         jpegDecodeToRGB(jpeg.data(),jpeg.size(),classified,w,h)){
        std::vector<uint8_t> texture((size_t)outW*outH*3);
        resizeNearestRGB(classified.data(),w,h,texture.data(),outW,outH);
        sendView(texture,stamp,101);lastTextureView=stamp;
      }
    }
  });
  CONSOLE.print("CAM out req="); CONSOLE.print(outW); CONSOLE.print("x"); CONSOLE.print(outH); CONSOLE.print(" @"); CONSOLE.println((int)fps);

  while (running_.load()) {
    // Drain completed-result counters on the capture thread: the logger is not
    // shared with the detector thread, and counts analyses rather than frames.
    for(unsigned n=detectionsFound.exchange(0);n;--n)lightLogger.detection(true);
    for(unsigned n=detectionsMissing.exchange(0);n;--n)lightLogger.detection(false);
    const auto freshnessNow=steady_clock::now();
    if(reflectorDetectionRequested_.load()){
      const auto marker=ReflectorStripsDetector::instance().observation(
          (unsigned long)duration_cast<milliseconds>(freshnessNow.time_since_epoch()).count());
      if(marker.ageMs!=(unsigned long)-1){
        maximumMarkerAge=std::max(maximumMarkerAge,marker.ageMs);
      }
      ++markerSamples;if(!marker.found)++staleMarkerSamples;
      if(freshnessNow-lastFreshnessReport>=seconds(5)){
        CONSOLE.print("CAM reflectors freshness maxAge=");CONSOLE.print(maximumMarkerAge);
        CONSOLE.print(" unavailable=");CONSOLE.print(staleMarkerSamples);
        CONSOLE.print(" samples=");CONSOLE.println(markerSamples);
        maximumMarkerAge=0;markerSamples=staleMarkerSamples=0;lastFreshnessReport=freshnessNow;
      }
    }
    int nIndex = captureIndex();
    videoWorker.setRate(reqFps_.load());
    if (nIndex != index) { CONSOLE.println("CAM index changed, reopening"); break; }

    // A disconnected or stalled camera must leave this session, rather than
    // retrying the same broken fd forever. DQBUF cannot block after select.
    v4l2_buffer b{};
    std::vector<uint8_t> simulatedJpeg;
    uint64_t sourceAgeMs=0;
    const uint8_t* mjpeg=nullptr;size_t mjpegLen=0;
    if(virtualCamera){
      const auto deadline=steady_clock::now()+seconds(2);
      while(running_.load() && !Sim3D::frame(simulatedJpeg,virtualSequence,sourceAgeMs) && steady_clock::now()<deadline)
        std::this_thread::sleep_for(milliseconds(10));
      if(!running_.load() || simulatedJpeg.empty()){CONSOLE.println("CAM simulator timeout");break;}
      mjpeg=simulatedJpeg.data();mjpegLen=simulatedJpeg.size();
    }else{
    fd_set fds;FD_ZERO(&fds);FD_SET(fd,&fds);
    timeval tv;tv.tv_sec=2;tv.tv_usec=0;
    const int r=select(fd+1,&fds,NULL,NULL,&tv);
    if(r<0 && errno==EINTR)continue;
    if(r<=0){
      if(r==0)lightLogger.timeout();
      CONSOLE.println("CAM capture timeout/error, reopening");break;
    }
    memset(&b,0,sizeof(b));b.type=req.type;b.memory=V4L2_MEMORY_MMAP;
    if(ioctl(fd,VIDIOC_DQBUF,&b)<0){
      if(errno==EAGAIN || errno==EINTR)continue;
      CONSOLE.println("CAM dequeue failed, reopening");break;
    }
    lightLogger.capture(b);
    if (b.index >= bufs.size()) { CONSOLE.println("CAM invalid buffer index, reopening"); break; }
    if ((b.flags & V4L2_BUF_FLAG_ERROR) || !b.bytesused || b.bytesused > bufs[b.index].length) {
      if (steady_clock::now() - lastBadFrameLog >= seconds(5)) {
        CONSOLE.println("CAM dropped invalid capture buffer"); lastBadFrameLog = steady_clock::now();
      }
      lightLogger.invalid();
      lightLogger.sample(fd,index,cameraGetNavigationMode());
      if(ioctl(fd,VIDIOC_QBUF,&b)<0){CONSOLE.println("CAM requeue failed, reopening");break;}
      continue;
    }
    mjpeg = (const uint8_t*)bufs[b.index].start; mjpegLen = b.bytesused;
    }
    auto now = steady_clock::now();
    const bool shouldSend = streamingRequested_.load();
    const int detectorFps = std::max(1, reflectorFps_.load());
    const auto detectorPeriod = milliseconds(1000 / detectorFps);
    const bool shouldDetect = reflectorDetectionRequested_.load() && (now - lastDetection) >= detectorPeriod;
    const bool shouldGrass=(grassMonitoringRequested_.load() || streamingRequested_.load()) && now-lastGrass>=milliseconds(100);
    const bool shouldMeasureLight=now-lastLightMeasurement>=seconds(1);
    if (shouldSend || shouldDetect || shouldGrass || lightLogger.due() || shouldMeasureLight) {
      std::vector<uint8_t> rgb; int srcW = 0, srcH = 0;
      if (jpegDecodeToRGB(mjpeg, mjpegLen, rgb, srcW, srcH)) {
        lightLogger.decoded();
        if(shouldMeasureLight){
          const auto light=CameraLightLogger::measure(rgb.data(),srcW,srcH);
          darkness.update(light.region,duration<double>(now.time_since_epoch()).count(),light.regionP80);
          lastLightMeasurement=now;
        }
        const unsigned long timestampMs = (unsigned long)duration_cast<milliseconds>(now.time_since_epoch()).count()-sourceAgeMs;
        if(shouldGrass){grass.submit(rgb.data(),srcW,srcH,timestampMs);lastGrass=now;}
        if (shouldDetect) {
          ReflectorFrame frame;
          frame.rgb=rgb;frame.jpeg.assign(mjpeg,mjpeg+mjpegLen);
          frame.width=srcW;frame.height=srcH;frame.timestampMs=timestampMs;
          frame.linear=cameraGetLinearSet();frame.angular=cameraGetAngularSet();
          reflectorWorker.submit(std::move(frame));
          lastDetection = now;
        }
        lightLogger.sample(fd,index,cameraGetNavigationMode(),rgb.data(),srcW,srcH,mjpeg,mjpegLen,
            reflectorDetectionRequested_.load()?ReflectorStripsDetector::instance().observation(timestampMs).confidence:0);
        // Share clean, fixed-resolution frames only while a viewer requests video.
        if (streamingRequested_.load() && now-lastTexture>=milliseconds(500)) {
          std::vector<uint8_t> clean(640*360*3), encoded;
          resizeNearestRGB(rgb.data(),srcW,srcH,clean.data(),640,360);
          if(jpegEncodeRGB(clean.data(),640,360,90,encoded))TextureFiles::publish(encoded,timestampMs);
          lastTexture=now;
        }
        if(streamingRequested_.load()){
          VideoFrame video;video.rgb=std::move(rgb);video.width=srcW;video.height=srcH;
          video.timestampMs=timestampMs;video.dark=darkness.dark();videoWorker.submit(std::move(video));
        }
      } else {
        lightLogger.invalid();
        lightLogger.sample(fd,index,cameraGetNavigationMode());
        if (now - lastBadFrameLog >= seconds(5)) {
          CONSOLE.println("CAM dropped invalid JPEG frame");
          lastBadFrameLog = now;
        }
      }
    }
    if(!virtualCamera && ioctl(fd,VIDIOC_QBUF,&b)<0){CONSOLE.println("CAM requeue failed, reopening");break;}
  }
  videoWorker.stop();
  reflectorWorker.stop();
}

extern "C" void cameraStreamerStart(int index, int width, int height, int fps, int quality, bool auxiliaryViews) { CameraStreamer::instance().start(index, width, height, fps, quality, auxiliaryViews); }
extern "C" void cameraStreamerStop() { CameraStreamer::instance().stop(); }
extern "C" void cameraReflectorStripsEnable(int index, int fps) { CameraStreamer::instance().enableReflectorStripsDetection(index, fps); }
extern "C" bool cameraReflectorStripsFound() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).found;
}
extern "C" float cameraReflectorStripsHorizontalSpan() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).horizontalSpan;
}
extern "C" float cameraReflectorStripsHorizontalError() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).horizontalError;
}
extern "C" float cameraReflectorStripsConfidence() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).confidence;
}
extern "C" int cameraReflectorStripsStripCount() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).stripCount;
}
extern "C" unsigned long cameraReflectorStripsAgeMs() {
  return ReflectorStripsDetector::instance().observation((unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()).ageMs;
}

void CameraStreamer::buildAndSendFrame() {
  Sender s; int w = reqW_.load(), h = reqH_.load(); { std::lock_guard<std::mutex> lk(mtx_); s = sender_; }
  if (!s) return;
  if (w < 1) w = 160; if (h < 1) h = 120;
  // Generate a solid red image WxH
  std::vector<uint8_t> rgb((size_t)w * (size_t)h * 3);
  for (size_t i = 0; i < (size_t)w * (size_t)h; i++) {
    rgb[i*3 + 0] = 255; // R
    rgb[i*3 + 1] = 0;   // G
    rgb[i*3 + 2] = 0;   // B
  }
  // Overlay on fallback frame as well
  drawOverlay(rgb.data(), w, h);
  const CameraFrameClock frameClock;
  const uint64_t frameTimestamp = CameraFrameClock::monotonicMs();
  drawText(rgb.data(),w,h,4,4,"CAM ERROR",1,255,255,255);
  drawFrameTimestamp(rgb.data(),w,h,frameClock.label(frameTimestamp));
  std::vector<uint8_t> jpeg;
  const int q = reqQ_.load();
  jpegEncodeRGB(rgb.data(), w, h, (q > 0 ? q : 70), jpeg);
  // Build header + send
  uint8_t hdr[16];
  uint32_t ts = uint32_t(frameTimestamp);
  buildMediaHeader(hdr, 1, (uint16_t)w, (uint16_t)h, ts, uint16_t(0x8000 | (camIndex_.load() & 0xff)));
  std::vector<uint8_t> buf(sizeof(hdr) + jpeg.size());
  std::memcpy(buf.data(), hdr, sizeof(hdr));
  if (!jpeg.empty()) std::memcpy(buf.data() + sizeof(hdr), jpeg.data(), jpeg.size());
  s(buf.data(), buf.size());
}

#endif // __linux__
