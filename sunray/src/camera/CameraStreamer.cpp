#include "camera/CameraStreamer.h"
#include "camera/MediaStream.h"
#include "Console.h"
// Default to LinuxConsole if CONSOLE macro is not mapped by a config
#ifndef CONSOLE
#define CONSOLE Console
#endif
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <jpeglib.h>

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
CameraStreamer::~CameraStreamer() { stop(); }

void CameraStreamer::setSender(Sender s) { std::lock_guard<std::mutex> lk(mtx_); sender_ = std::move(s); }

void CameraStreamer::start(int index, int width, int height, int fps) {
  camIndex_.store(index);
  reqW_.store(width);
  reqH_.store(height);
  reqFps_.store((fps <= 0) ? 5 : fps);
  if (running_.load()) return;
  running_.store(true);
  CONSOLE.print("CAM start idx="); CONSOLE.print(index);
  CONSOLE.print(" req="); CONSOLE.print(width); CONSOLE.print("x"); CONSOLE.print(height);
  CONSOLE.print(" @"); CONSOLE.println((int)fps);
  worker_ = std::thread([this]() { runLoop(); });
}

void CameraStreamer::stop() {
  if (!running_.load()) return;
  running_.store(false);
  try { if (worker_.joinable()) worker_.join(); } catch (...) {}
  CONSOLE.println("CAM stop");
}

struct MMapBuffer { void* start{nullptr}; size_t length{0}; };

static bool jpegDecodeToRGB(const uint8_t* data, size_t len, std::vector<uint8_t>& outRGB, int& w, int& h) {
  jpeg_decompress_struct cinfo; jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, const_cast<unsigned char*>(data), len);
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) { jpeg_destroy_decompress(&cinfo); return false; }
  jpeg_start_decompress(&cinfo);
  w = cinfo.output_width; h = cinfo.output_height; int ch = cinfo.output_components; if (ch != 3) { jpeg_destroy_decompress(&cinfo); return false; }
  outRGB.resize((size_t)w * (size_t)h * 3);
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW rowptr = (JSAMPROW)&outRGB[(size_t)cinfo.output_scanline * (size_t)w * 3];
    jpeg_read_scanlines(&cinfo, &rowptr, 1);
  }
  jpeg_finish_decompress(&cinfo); jpeg_destroy_decompress(&cinfo); return true;
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
  // Open V4L2 device
  int index = camIndex_.load(); int outW = reqW_.load(); int outH = reqH_.load(); int fps = reqFps_.load();
  char devPath[64]; std::snprintf(devPath, sizeof(devPath), "/dev/video%d", index);
  int fd = ::open(devPath, O_RDWR);
  if (fd < 0) {
    CONSOLE.print("CAM open failed: "); CONSOLE.println(devPath);
    auto period = (fps > 0) ? milliseconds(1000 / fps) : milliseconds(200);
    auto next = steady_clock::now();
    while (running_.load()) { buildAndSendFrame(); next += period; std::this_thread::sleep_until(next); }
    return;
  }
  CONSOLE.print("CAM opened "); CONSOLE.println(devPath);
  // Try to set format to MJPEG
  v4l2_format fmt; memset(&fmt, 0, sizeof(fmt)); fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) { fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; ioctl(fd, VIDIOC_S_FMT, &fmt); }
  CONSOLE.print("CAM source "); CONSOLE.print((int)fmt.fmt.pix.width); CONSOLE.print("x"); CONSOLE.println((int)fmt.fmt.pix.height);
  // Request MMAP
  v4l2_requestbuffers req; memset(&req, 0, sizeof(req)); req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) { CONSOLE.println("CAM reqbufs failed"); ::close(fd); return; }
  std::vector<MMapBuffer> bufs(req.count);
  for (unsigned i = 0; i < req.count; i++) {
    v4l2_buffer b; memset(&b, 0, sizeof(b)); b.type = req.type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { CONSOLE.println("CAM querybuf failed"); ::close(fd); return; }
    void* start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (start == MAP_FAILED) { CONSOLE.println("CAM mmap failed"); ::close(fd); return; }
    bufs[i].start = start; bufs[i].length = b.length;
  }
  for (unsigned i = 0; i < req.count; i++) { v4l2_buffer b; memset(&b, 0, sizeof(b)); b.type = req.type; b.memory = V4L2_MEMORY_MMAP; b.index = i; ioctl(fd, VIDIOC_QBUF, &b); }
  int type = req.type; ioctl(fd, VIDIOC_STREAMON, &type);

  auto lastSend = steady_clock::now() - milliseconds(1000);
  auto minPeriod = (fps > 0) ? milliseconds(1000 / fps) : milliseconds(200);
  CONSOLE.print("CAM out req="); CONSOLE.print(outW); CONSOLE.print("x"); CONSOLE.print(outH); CONSOLE.print(" @"); CONSOLE.println((int)fps);

  while (running_.load()) {
    int nIndex = camIndex_.load(), nW = reqW_.load(), nH = reqH_.load(), nF = reqFps_.load();
    if (nIndex != index) { CONSOLE.println("CAM index changed, reopening"); break; }
    outW = nW; outH = nH; if (nF != fps) { fps = nF; minPeriod = (fps > 0) ? milliseconds(1000 / fps) : milliseconds(200); }

    fd_set fds; FD_ZERO(&fds); FD_SET(fd, &fds); timeval tv; tv.tv_sec = 1; tv.tv_usec = 0; int r = select(fd + 1, &fds, NULL, NULL, &tv); if (r <= 0) continue;
    v4l2_buffer b; memset(&b, 0, sizeof(b)); b.type = req.type; b.memory = V4L2_MEMORY_MMAP; if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) continue;
    const uint8_t* mjpeg = (const uint8_t*)bufs[b.index].start; size_t mjpegLen = b.bytesused;
    auto now = steady_clock::now(); bool shouldSend = (now - lastSend) >= minPeriod;
    if (shouldSend) {
      std::vector<uint8_t> rgb; int srcW = 0, srcH = 0;
      if (jpegDecodeToRGB(mjpeg, mjpegLen, rgb, srcW, srcH)) {
        std::vector<uint8_t> outRGB((size_t)outW * (size_t)outH * 3);
        resizeNearestRGB(rgb.data(), srcW, srcH, outRGB.data(), outW, outH);
        std::vector<uint8_t> outJpeg; jpegEncodeRGB(outRGB.data(), outW, outH, 70, outJpeg);
        Sender s; { std::lock_guard<std::mutex> lk(mtx_); s = sender_; }
        if (s) {
          uint8_t hdr[16]; uint32_t ts = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() & 0xFFFFFFFFu);
          buildMediaHeader(hdr, 1, (uint16_t)outW, (uint16_t)outH, ts, 0);
          std::vector<uint8_t> buf(sizeof(hdr) + outJpeg.size()); std::memcpy(buf.data(), hdr, sizeof(hdr)); std::memcpy(buf.data() + sizeof(hdr), outJpeg.data(), outJpeg.size());
          s(buf.data(), buf.size());
        }
      }
      lastSend = now;
    }
    ioctl(fd, VIDIOC_QBUF, &b);
  }
  ioctl(fd, VIDIOC_STREAMOFF, &type);
  for (auto& mb : bufs) { if (mb.start && mb.length) munmap(mb.start, mb.length); }
  ::close(fd);
  if (running_.load()) { runLoop(); }
}

extern "C" void cameraStreamerStart(int index, int width, int height, int fps) { CameraStreamer::instance().start(index, width, height, fps); }
extern "C" void cameraStreamerStop() { CameraStreamer::instance().stop(); }

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
  std::vector<uint8_t> jpeg;
  jpegEncodeRGB(rgb.data(), w, h, 70, jpeg);
  // Build header + send
  uint8_t hdr[16];
  uint32_t ts = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFFu);
  buildMediaHeader(hdr, 1, (uint16_t)w, (uint16_t)h, ts, 0);
  std::vector<uint8_t> buf(sizeof(hdr) + jpeg.size());
  std::memcpy(buf.data(), hdr, sizeof(hdr));
  if (!jpeg.empty()) std::memcpy(buf.data() + sizeof(hdr), jpeg.data(), jpeg.size());
  s(buf.data(), buf.size());
}
