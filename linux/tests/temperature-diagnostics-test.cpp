#include <cassert>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include "../../sunray/src/ublox/TemperatureDiagnostics.h"
#include "../../sunray/src/linux/CpuTemperatureDiagnostics.h"

struct Output {
  std::ostringstream text;
  template<class T> void print(T value) { text << value; }
  void print(float value, int digits) { text << std::fixed << std::setprecision(digits) << value; }
  template<class T> void println(T value) { print(value); println(); }
  void println() { text << '\n'; }
};
static void zone(const std::string& root, int number, const char* type, const char* temp) {
  std::string path = root + "/thermal_zone" + std::to_string(number);
  mkdir(path.c_str(), 0700);
  std::ofstream(path + "/type") << type;
  std::ofstream(path + "/temp") << temp;
}
int main(int argc, char** argv) {
  assert(argc == 2);
  UbloxTemperatureDiagnostics d;
  Output missing; d.log(missing, 1);
  assert(missing.text.str().find("unavailable") != std::string::npos);
  char packet[24] = {}; packet[0] = 1; packet[18] = 53;
  assert(d.update(packet, sizeof(packet), 100));
  assert(d.temperature == 53);
  packet[18] = static_cast<char>(246);
  assert(d.update(packet, sizeof(packet), 200));
  assert(d.temperature == -10);
  assert(!d.update(packet, 18, 300));
  packet[0] = 2; assert(!d.update(packet, 24, 300));
  assert(d.receivedAt == 200 && d.temperature == -10);
  Output regular; d.log(regular, 70200);
  assert(regular.text.str().find("STALE") == std::string::npos);
  Output stale; d.log(stale, 150201);
  assert(stale.text.str().find("STALE") != std::string::npos);
  packet[0] = 1; assert(d.update(packet, 24, UINT32_MAX - 100));
  Output wrap; d.log(wrap, 100);
  assert(wrap.text.str().find("ageMs=201") != std::string::npos);
  assert(wrap.text.str().find("STALE") == std::string::npos);

  const std::string root = argv[1];
  Output none; logCpuTemperatureDiagnostics(none, root.c_str());
  assert(none.text.str().find("unavailable") != std::string::npos);
  zone(root, 0, "gpu-thermal", "90000");
  zone(root, 1, "soc-thermal", "45000");
  zone(root, 2, "bigcore0-thermal", "52300");
  zone(root, 3, "cpu-thermal", "invalid");
  zone(root, 4, "littlecore-thermal", "2147483647");
  Output cpu; logCpuTemperatureDiagnostics(cpu, root.c_str());
  assert(cpu.text.str() == "CPU-TEMP[C]=52.3 sensor=bigcore0-thermal\n");
  std::cout << "PASS temperature: signed MON-SYS, malformed frames, stale/wraparound, CPU sensor selection and unavailable/invalid readings\n";
}
