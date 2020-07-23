// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/* see Wiki for installation details:
   http://wiki.ardumower.de/index.php?title=Ardumower_Sunray

   requirements:
   + Ardumower chassis and Ardumower motors
   + Ardumower PCB 1.3
   +   Arduino Due
   +   Ardumower BLE UART module (HM-10/CC2540/CC2541)
   +   optional: Ardumower IMU (MPU6050/MPU9150/MPU9250/MPU9255) - choose your IMU below
   +   optional: Ardumower WIFI (ESP8266 ESP-01 with stock firmware)
   +   optional: HTU21D temperature/humidity sensor
   + Ardumower RTK (ublox F9P)


1. Rename file 'config_example.h' into 'config.h'

2. Configure the options below and finally compile and upload this project.

Arduino UPLOAD NOTE:

If using Arduino Due 'native' USB port for uploading, choose board 'Arduino Due native' in the
Arduino IDE and COM port 'Arduino Due native port'.

If using Arduino Due 'programming' USB port for uploading, choose board 'Arduino Due programming' in the
Arduino IDE and COM port 'Arduino Due programming port'.

Also, you may choose the serial port below for serial monitor output (CONSOLE).

*/

#ifdef __cplusplus
  #include "udpserial.h"
#endif


// ------- Bluetooth4.0/BLE module -----------------------------------
// see Wiki on how to install the BLE module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module
// ------- RTK GPS module -----------------------------------
// see Wiki on how to install the GPS module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module

// -------- IMU sensor  ----------------------------------------------
// choose one MPU IMU (make sure to connect AD0 on the MPU board to 3.3v)
// verify in CONSOLE that your IMU was found (you will hear 8 buzzer beeps for automatic calibration at start)
// see Wiki for wiring, how to position the module, and to configure the I2C pullup jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#IMU.2C_sensor_fusion

//#define MPU6050
//#define MPU9150
#define MPU9250   // also choose this for MPU9255


// should the mower turn off if IMU is tilt over? (yes: uncomment line, no: comment line)
#define ENABLE_TILT_DETECTION  1

// --------- serial monitor output (CONSOLE) ------------------------
// which Arduino Due USB port do you want to your for serial monitor output (CONSOLE)?
// Arduino Due native USB port  => choose SerialUSB
// Arduino Due programming port => choose Serial
#define CONSOLE SerialUSB   // do not change
//#define CONSOLE Serial
//#define CONSOLE udpSerial

// ------- serial ports and baudrates---------------------------------
#define CONSOLE_BAUDRATE    115200    // baudrate used for console
//#define CONSOLE_BAUDRATE    921600  // baudrate used for console
#define BLE_BAUDRATE    115200        // baudrate used for BLE
#define GPS_BAUDRATE  115200          // baudrate for GPS RTK module
#define WIFI_BAUDRATE 115200          // baudrate for WIFI module

#define WIFI Serial1
#define BLE Serial2
#define GPS Serial3

// NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:
// 1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom
// 2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h
// change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024


// ------ odometry -----------------------------------
// values below are for Ardumower chassis and Ardumower motors
// see Wiki on how to configure the odometry divider:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#PCB1.3_odometry_divider
// NOTE: It is important to verify your odometry is working accurate.
// Follow the steps described in the Wiki to verify your odometry returns approx. 1 meter distance for
// driving the same distance on the ground (without connected GPS):
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Odometry_test
#define WHEEL_BASE_CM         36         // wheel-to-wheel distance (cm)
#define WHEEL_DIAMETER        250        // wheel diameter (mm)


// choose ticks per wheel revolution :
// ...for the 36mm diameter motor (blue cap)  https://www.marotronics.de/2-x-36er-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle
//#define TICKS_PER_REVOLUTION  1310 / 2    // odometry ticks per wheel revolution

// ...for the 36mm diameter motor (black cap)  https://www.marotronics.de/MA36-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
// #define TICKS_PER_REVOLUTION 975 / 2

// ...for the newer 42mm diameter motor (green connector) https://www.marotronics.de/MA42-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
#define TICKS_PER_REVOLUTION  696 / 2    // odometry ticks per wheel revolution

// ...for the older 42mm diameter motor (white connector)  https://wiki.ardumower.de/images/d/d6/Ardumower_chassis_inside_ready.jpg
//#define TICKS_PER_REVOLUTION  1050 / 2    // odometry ticks per wheel revolution



// ----- mowing motor -------------------------------------------------
// should the direction of mowing motor toggle each start? (yes: true, no: false)
#define MOW_TOGGLE_DIR       true
//#define MOW_TOGGLE_DIR       false

// should the motor overload detection be enabled?
#define ENABLE_OVERLOAD_DETECTION  true
//#define ENABLE_OVERLOAD_DETECTION  false

// should the motor fault (error) detection be enabled?
//#define ENABLE_FAULT_DETECTION  true
#define ENABLE_FAULT_DETECTION  false      // use this if you keep getting 'motor error'


// ------ WIFI module (ESP8266 ESP-01) --------------------------------
// WARNING: WIFI is highly experimental - not for productive use (yet)
// NOTE: all settings (maps, absolute position source etc.) are stored in your phone - when using another
// device for the WIFI connection (PC etc.), you will have to transfer those settings (share maps via app,
// re-enter absolute position source etc) !
// see Wiki on how to install the WIFI module and configure the WIFI jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module

#define START_AP  false             // should WIFI module start its own access point?
//#define WIFI_IP   192,168,2,15      // choose IP e.g. 192.168.4.1  (comment out for dynamic IP/DHCP)
#define WIFI_SSID "test"            // choose WiFi network ID
#define WIFI_PASS "test"            // choose WiFi network password

#define ENABLE_SERVER true          // must be enabled for web app
//#define ENABLE_SERVER false

//#define ENABLE_UDP true                // enable console for UDP?
#define ENABLE_UDP false
#define UDP_SERVER_IP   192,168,2,56     // remote UDP IP and port to connect to
#define UDP_SERVER_PORT 4210


// ------ ultrasonic sensor -----------------------------
//#define SONAR_ENABLE true
#define SONAR_ENABLE false
#define SONAR_OBSTACLE_CM 12      // stop mowing operation below this distance (cm)


// ----- battery charging current measurement (INA169) --------------
// the Marotronics charger outputs max 1.5A 
// ( https://www.marotronics.de/Ladegeraete-fuer-den-Ardumower-Akkus-24V-mit-Status-LED-auch-fuer-Li-Ion-Akkus )
// so we are using the INA169 in non-bridged mode (max. 2.5A)
// ( https://www.marotronics.de/INA169-Analog-DC-Current-Sensor-Breakout-60V-25A-5A-Marotronics )

#define CURRENT_FACTOR 0.5     // (non-bridged INA169, max. 2.5A)
//#define CURRENT_FACTOR 1.0   // (bridged INA169, max. 5A)


// ------ experimental options -------------------------
// detect robot being kidnapped?
#define KIDNAP_DETECT true
//#define KIDNAP_DETECT false

// drive curves smoothly?
//#define SMOOTH_CURVES  true
#define SMOOTH_CURVES  false

// require valid GPS signal all time? mower will go into error (fix timeout) if no valid GPS signal during mowing
#define REQUIRE_VALID_GPS  true
//#define REQUIRE_VALID_GPS  false

#define ENABLE_PATH_FINDER  true     // path finder is experimental (can be slow - you may have to wait until robot actually starts)
//#define ENABLE_PATH_FINDER  false



// ------- I2C addresses -----------------------------
#define DS1307_ADDRESS B1101000
#define AT24C32_ADDRESS B1010000


// ------- PCB1.3/Due settings -------------------------
#define IOREF 3.3   // I/O reference voltage

// ------ hardware pins---------------------------------------
// no configuration needed here
#define pinMotorEnable  37         // EN motors enable
#define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
#define pinMotorLeftSense A1       // M1_FB  left motor current sense
#define pinMotorLeftFault 25       // M1_SF  left motor fault

#define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
#define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
#define pinMotorRightSense A0      // M2_FB  right motor current sense
#define pinMotorRightFault 27      // M2_SF  right motor fault

#define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define pinMotorMowSense A3        // M1_FB  mower motor current sense
#define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowRpm A11

#define pinBumperLeft 39           // bumper pins
#define pinBumperRight 38

#define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
#define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

#define pinSonarCenterTrigger 24   // ultrasonic sensor pins
#define pinSonarCenterEcho 22
#define pinSonarRightTrigger 30
#define pinSonarRightEcho 32
#define pinSonarLeftTrigger 34
#define pinSonarLeftEcho 36
#define pinPerimeterRight A4       // perimeter
#define pinDockingReflector A4     // docking IR reflector
#define pinPerimeterLeft A5

#define pinLED 13                  // LED
#define pinBuzzer 53               // Buzzer
#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
#define pinButton 51               // digital ON/OFF button
#define pinBatteryVoltage A2       // battery voltage sensor
#define pinBatterySwitch 4         // battery-OFF switch
#define pinChargeVoltage A9        // charging voltage sensor
#define pinChargeCurrent A8        // charge current sensor
#define pinChargeRelay 50          // charge relay
#define pinRemoteMow 12            // remote control mower motor
#define pinRemoteSteer 11          // remote control steering
#define pinRemoteSpeed 10          // remote control speed
#define pinRemoteSwitch 52         // remote control switch
#define pinVoltageMeasurement A7   // test pin for your own voltage measurements
#ifdef __AVR__
  #define pinOdometryLeft A12      // left odometry sensor
  #define pinOdometryLeft2 A13     // left odometry sensor (optional two-wire)
  #define pinOdometryRight A14     // right odometry sensor
  #define pinOdometryRight2 A15    // right odometry sensor (optional two-wire)
#else
  #define pinOdometryLeft DAC0     // left odometry sensor
  #define pinOdometryLeft2 DAC1    // left odometry sensor (optional two-wire)
  #define pinOdometryRight CANRX   // right odometry sensor
  #define pinOdometryRight2 CANTX  // right odometry sensor (optional two-wire)
#endif
#define pinLawnFrontRecv 40        // lawn sensor front receive
#define pinLawnFrontSend 41        // lawn sensor front sender
#define pinLawnBackRecv 42         // lawn sensor back receive
#define pinLawnBackSend 43         // lawn sensor back sender
#define pinUserSwitch1 46          // user-defined switch 1
#define pinUserSwitch2 47          // user-defined switch 2
#define pinUserSwitch3 48          // user-defined switch 3
#define pinRain 44                 // rain sensor

// IMU (compass/gyro/accel): I2C  (SCL, SDA)
// Bluetooth: Serial2 (TX2, RX2)
// GPS: Serial3 (TX3, RX3)
// WIFI: Serial1 (TX1, RX1)

#define DEBUG(x) CONSOLE.print(x)
#define DEBUGLN(x) CONSOLE.println(x)
