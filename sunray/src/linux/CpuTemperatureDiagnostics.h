#ifndef SUNRAY_CPU_TEMPERATURE_DIAGNOSTICS_H
#define SUNRAY_CPU_TEMPERATURE_DIAGNOSTICS_H
#include <stdio.h>
#include <string.h>
#include <dirent.h>

// Read CPU/SoC thermal zones, never assume thermal_zone0 is the CPU.
// On RK3588 this includes soc-thermal, bigcore*-thermal and littlecore-thermal.
// Report the hottest CPU/SoC sensor, excluding GPU/NPU sensors. No subprocesses.
template<class Output>
void logCpuTemperatureDiagnostics(Output& out, const char* root = "/sys/class/thermal") {
  DIR* dir = opendir(root);
  bool found = false;
  long hottest = -100000;
  char hottestType[128] = {};
  if (dir) {
    struct dirent* entry;
    while ((entry = readdir(dir))) {
      if (strncmp(entry->d_name, "thermal_zone", 12) != 0) continue;
      char path[1024], type[128] = {};
      snprintf(path, sizeof(path), "%s/%s/type", root, entry->d_name);
      FILE* file = fopen(path, "r");
      if (!file) continue;
      const bool haveType = fscanf(file, "%127s", type) == 1;
      fclose(file);
      if (!haveType || !(strstr(type, "cpu") || strstr(type, "soc") ||
          strstr(type, "bigcore") || strstr(type, "littlecore") ||
          strcmp(type, "x86_pkg_temp") == 0)) continue;
      snprintf(path, sizeof(path), "%s/%s/temp", root, entry->d_name);
      file = fopen(path, "r");
      if (!file) continue;
      long milliC = 0;
      const bool valid = fscanf(file, "%ld", &milliC) == 1;
      fclose(file);
      if (!valid || milliC < -60000 || milliC > 150000) continue;
      if (!found || milliC > hottest) {
        hottest = milliC;
        strcpy(hottestType, type);
        found = true;
      }
    }
    closedir(dir);
  }
  out.print("CPU-TEMP[C]=");
  if (!found) {
    out.println("unavailable (no readable CPU/SoC thermal sensor)");
    return;
  }
  out.print(hottest / 1000.0f, 1);
  out.print(" sensor="); out.println(hottestType);
}
#endif
