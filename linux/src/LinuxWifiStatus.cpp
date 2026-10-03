#include "LinuxWifiStatus.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace {
LinuxWifiStatus queryWifiStatus() {
  LinuxWifiStatus result;
  // This command only reads connection status. Bound a stalled supplicant and
  // run it exclusively on the status thread, never on the robot control loop.
  FILE* pipe = popen("timeout -k 1s 2s wpa_cli -i wlan0 status 2>/dev/null", "r");
  if (!pipe) return result;
  char line[256];
  while (fgets(line, sizeof(line), pipe)) {
    line[strcspn(line, "\r\n")] = '\0';
    if (strcmp(line, "wpa_state=COMPLETED") == 0) result.connected = true;
    if (strcmp(line, "wpa_state=INACTIVE") == 0) result.inactive = true;
  }
  if (pclose(pipe) != 0) return {}; // A failed query must not retain connected.
  return result;
}

class WifiStatusWorker {
public:
  explicit WifiStatusWorker(std::function<LinuxWifiStatus()> query = queryWifiStatus,
                            unsigned intervalMs = 5000)
    : query(std::move(query)), intervalMs(intervalMs), thread([this]{ run(); }) {}
  ~WifiStatusWorker() {
    { std::lock_guard<std::mutex> lock(mutex); stop = true; }
    wake.notify_all();
    thread.join();
  }
  LinuxWifiStatus snapshot() const {
    const unsigned value = status.load();
    return {bool(value & 1), bool(value & 2)};
  }
private:
  std::function<LinuxWifiStatus()> query;
  unsigned intervalMs;
  std::atomic<unsigned> status{0};
  std::mutex mutex;
  std::condition_variable wake;
  bool stop = false;
  std::thread thread; // All state must be initialized before starting the thread.
  void run() {
    for (;;) {
      auto value = query();
      status.store((value.connected ? 1u : 0u) | (value.inactive ? 2u : 0u));
      std::unique_lock<std::mutex> lock(mutex);
      if (wake.wait_for(lock, std::chrono::milliseconds(intervalMs), [this]{return stop;})) return;
    }
  }
};
}

LinuxWifiStatus linuxWifiStatus() {
  static WifiStatusWorker worker;
  return worker.snapshot();
}
