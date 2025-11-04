// Avoid Arduino-style min/max macro conflicts with C++ std headers
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#pragma once
#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <mutex>

class CameraStreamer {
public:
  using Sender = std::function<void(const uint8_t* data, size_t len)>;
  static CameraStreamer& instance();

  void setSender(Sender s);
  void start(int index, int width, int height, int fps);
  void stop();
  bool running() const { return running_.load(); }

private:
  CameraStreamer();
  ~CameraStreamer();
  CameraStreamer(const CameraStreamer&) = delete;
  CameraStreamer& operator=(const CameraStreamer&) = delete;

  void runLoop();
  void buildAndSendFrame();

  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex mtx_;
  Sender sender_;
  std::atomic<int> camIndex_{0};
  std::atomic<int> reqW_{160};
  std::atomic<int> reqH_{120};
  std::atomic<int> reqFps_{5};
};

