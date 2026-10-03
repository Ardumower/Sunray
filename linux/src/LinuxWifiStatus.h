#pragma once

// A cached display/LED status. Reading this never waits for wpa_supplicant.
struct LinuxWifiStatus {
  bool connected = false;
  bool inactive = false;
};

LinuxWifiStatus linuxWifiStatus();
