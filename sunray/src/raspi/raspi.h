// Raspberry PI


#if defined(__linux__)

#include <LinuxSerial.h>
#include <BleUartServer.h>

extern LinuxSerial SerialROBOT;
extern LinuxSerial SerialWIFI;
extern BleUartServer SerialBLE;
extern LinuxSerial SerialGPS;


#endif  // __linux__

