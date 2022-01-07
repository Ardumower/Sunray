#include "../../config.h"

#if defined(__linux__)

LinuxSerial SerialROBOT(SERIAL_ROBOT_PATH); // serial robot
LinuxSerial SerialWIFI(SERIAL_WIFI_PATH); // WIFI
BleUartServer SerialBLE; // BLE 
LinuxSerial SerialGPS(SERIAL_GPS_PATH); // GPS
LinuxSerial SerialNTRIP(SERIAL_NTRIP_PATH); // NTRIP



#endif   //  __linux__

