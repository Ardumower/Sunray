#ifndef UBLOX_SIGNAL_DIAGNOSTICS_H
#define UBLOX_SIGNAL_DIAGNOSTICS_H
#include <stdint.h>
#include <stddef.h>

// NAV-SIG contains separate entries for each satellite/frequency signal.
// These diagnostics include unused/weak signals, independently of fix filters.
struct UbloxSignalDiagnostics {
  struct Signal { uint8_t gnss, sv, signal, cno; };
  bool available = false;
  uint32_t receivedAt = 0;
  uint16_t sum = 0;
  uint8_t reported = 0, nonzero = 0, used = 0, maximum = 0, topCount = 0;
  Signal top[8] = {};

  bool update(const char* data, size_t length, uint32_t now) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    if (length < 8 || p[4] != 0 || length != size_t(8 + 16 * p[5])) return false;
    *this = UbloxSignalDiagnostics();
    available = true;
    receivedAt = now;
    reported = p[5];
    for (unsigned i = 0; i < reported; ++i) {
      const uint8_t* s = p + 8 + 16*i;
      Signal value = {s[0], s[1], s[2], s[6]};
      if (s[10] & 8) ++used; // pseudorange used in navigation solution
      if (!value.cno) continue;
      ++nonzero;
      sum += value.cno;
      if (value.cno > maximum) maximum = value.cno;
      unsigned at = 0;
      while (at < topCount && top[at].cno >= value.cno) ++at;
      if (at >= 8) continue;
      if (topCount < 8) ++topCount;
      for (unsigned j = topCount-1; j > at; --j) top[j] = top[j-1];
      top[at] = value;
    }
    return true;
  }
};
#endif
