// Arduino Framework for Linux


#if defined(__linux__)

#include <LinuxSerial.h>
#include <BleUartServer.h>

extern LinuxSerial SerialROBOT;
extern LinuxSerial SerialWIFI;
extern BleUartServer SerialBLE;
extern LinuxSerial SerialGPS;
extern LinuxSerial SerialNTRIP;


#endif  // __linux__

