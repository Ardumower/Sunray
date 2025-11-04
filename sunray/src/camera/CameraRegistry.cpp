#ifdef __linux__
#include "camera/CameraRegistry.h"
#include "Console.h"
#include <time.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#ifndef CONSOLE
#define CONSOLE Console
#endif

static std::vector<CameraInfo> g_list;
static unsigned long g_lastRefreshMs = 0;

static unsigned long millis_linux() {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long)(ts.tv_sec * 1000ul + ts.tv_nsec / 1000000ul);
}

namespace CameraRegistry {
  void refresh() {
    const unsigned long now = millis_linux();
    if (g_lastRefreshMs && (now - g_lastRefreshMs) < 2000) return; // throttle
    g_list.clear();
    for (int i = 0; i < 16; i++) {
      char dev[64]; std::snprintf(dev, sizeof(dev), "/dev/video%d", i);
      int fd = ::open(dev, O_RDWR);
      if (fd < 0) continue;
      v4l2_capability cap; std::memset(&cap, 0, sizeof(cap));
      if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        if (cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
          v4l2_format fmt; std::memset(&fmt, 0, sizeof(fmt)); fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
            int w = (int)fmt.fmt.pix.width;
            int h = (int)fmt.fmt.pix.height;
            if (w > 0 && h > 0) {
              CameraInfo ci; ci.path = dev; ci.width = w; ci.height = h;
              g_list.push_back(ci);
            }
          }
        }
      }
      ::close(fd);
    }
    CONSOLE.print("CAM list: " ); CONSOLE.println((int)g_list.size());
    for (size_t k = 0; k < g_list.size(); k++) {
      CONSOLE.print("  ["); CONSOLE.print((int)k); CONSOLE.print("] " );
      CONSOLE.print(g_list[k].path.c_str()); CONSOLE.print(" " );
      CONSOLE.print(g_list[k].width); CONSOLE.print("x"); CONSOLE.println(g_list[k].height);
    }
    g_lastRefreshMs = now;
  }

  const std::vector<CameraInfo>& list() {
    if (g_list.empty()) refresh();
    return g_list;
  }

  std::string pathForIndex(int idx) {
    if (g_list.empty()) refresh();
    if (idx < 0 || (size_t)idx >= g_list.size()) return std::string();
    return g_list[(size_t)idx].path;
  }
}
#endif
