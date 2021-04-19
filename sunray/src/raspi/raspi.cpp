#include "../../config.h"

#if defined(__linux__)

LinuxSerial Serial1("/dev/ttyUSB0"); // serial robot
LinuxSerial Serial2("/dev/null"); // WIFI
BleUartServer Serial3; // BLE 
LinuxSerial Serial4("/dev/ttyACM0"); // GPS



#endif   //  __linux__

