#include "../../config.h"

#if defined(__linux__)

LinuxSerial Serial1("/dev/ttyUSB1"); // NGP
LinuxSerial Serial2("/dev/null"); // WIFI
LinuxSerial Serial3("/dev/null"); // BLE 
LinuxSerial Serial4("/dev/null"); // GPS



#endif   //  __linux__

