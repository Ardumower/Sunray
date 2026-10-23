#ifndef SUNRAY_UBLOX_TEMPERATURE_DIAGNOSTICS_H
#define SUNRAY_UBLOX_TEMPERATURE_DIAGNOSTICS_H
#include <stdint.h>
#include <stddef.h>

struct UbloxTemperatureDiagnostics {
  bool available = false;
  int temperature = 0;
  uint32_t receivedAt = 0;

  // UBX-MON-SYS v1 (0x0a/0x39), signed degrees C at payload offset 18.
  // u-blox F9 HPG 1.32 Interface Description, section 3.14.13.
  bool update(const char* payload, size_t size, uint32_t now) {
    if (size != 24 || static_cast<unsigned char>(payload[0]) != 1) return false;
    const unsigned value = static_cast<unsigned char>(payload[18]);
    temperature = value < 128 ? int(value) : int(value) - 256;
    receivedAt = now;
    available = true;
    return true;
  }

  template<class Output> void log(Output& out, uint32_t now) const {
    out.print("F9P-TEMP[C]=");
    if (!available) {
      out.println("unavailable (no valid MON-SYS)");
      return;
    }
    out.print(temperature);
    const uint32_t age = now - receivedAt;
    out.print(" ageMs="); out.print(static_cast<unsigned long>(age));
    // MON-COMMS can arrive every 70 seconds; allow two logging cycles.
    if (age > 150000) out.print(" STALE");
    out.println();
  }
};
#endif
