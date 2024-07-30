// Arduino Framework for Linux


#if defined(__linux__)

#include <LinuxSerial.h>
#include <BleUartServer.h>

extern LinuxSerial SerialROBOT;
extern LinuxSerial SerialWIFI;
#ifdef LINUX_BLE
  extern BleUartServer SerialBLE;
#else
  extern LinuxSerial SerialBLE;
#endif
extern LinuxSerial SerialGPS;
extern LinuxSerial SerialNTRIP;


extern "C" {
  const char *getCPUArchitecture(); //Get current architecture, detectx nearly every architecture. 
}

#endif  // __linux__

