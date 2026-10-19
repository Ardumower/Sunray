#ifndef SUNRAY_WIFI_DIAGNOSTICS_H
#define SUNRAY_WIFI_DIAGNOSTICS_H

#ifdef __linux__
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>

// Read driver statistics only: no scans, subprocesses or network changes.
// Use the driver's validity/unit flags rather than treating relative levels as dBm.
template<class Output>
void logWifiDiagnostics(Output& out) {
  FILE* file = fopen("/proc/net/wireless", "r");
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (!file || fd < 0) {
    if (file) fclose(file);
    if (fd >= 0) close(fd);
    out.println("WLAN-SIGNAL unavailable (statistics inaccessible)");
    return;
  }
  char line[512];
  bool found = false;
  while (fgets(line, sizeof(line), file)) {
    char* colon = strchr(line, ':');
    if (!colon) continue;
    *colon = '\0';
    char* name = line;
    while (*name == ' ' || *name == '\t') ++name;
    size_t length = strlen(name);
    if (!length || length >= IFNAMSIZ) continue;
    found = true;
    out.print("WLAN-SIGNAL iface="); out.print(name);
    struct iwreq request = {};
    memcpy(request.ifr_name, name, length + 1);
    bool associated = false;
    if (ioctl(fd, SIOCGIWAP, &request) == 0) {
      for (int i = 0; i < 6; ++i) associated |= request.u.ap_addr.sa_data[i] != 0;
      if (!associated) {
        out.println(" unavailable (not associated)");
        continue;
      }
    }
    struct iw_statistics stats = {};
    request.u.data.pointer = &stats;
    request.u.data.length = sizeof(stats);
    request.u.data.flags = 0; // Do not clear updated flags for other readers.
    if (ioctl(fd, SIOCGIWSTATS, &request) < 0) {
      out.println(" unavailable (driver statistics unsupported)");
      continue;
    }
    if (!(stats.qual.updated & IW_QUAL_LEVEL_INVALID) &&
        (stats.qual.updated & IW_QUAL_DBM)) {
      out.print(" rssi[dBm]=");
      out.print(stats.qual.level >= 64 ? int(stats.qual.level) - 256 : int(stats.qual.level));
    } else {
      out.print(" rssi[dBm]=unavailable");
    }
    if (!(stats.qual.updated & IW_QUAL_NOISE_INVALID) &&
        (stats.qual.updated & IW_QUAL_DBM)) {
      out.print(" noise[dBm]=");
      out.print(stats.qual.noise >= 64 ? int(stats.qual.noise) - 256 : int(stats.qual.noise));
    }
    out.println();
  }
  fclose(file);
  close(fd);
  if (!found) out.println("WLAN-SIGNAL unavailable (no wireless interface)");
}
#endif
#endif
