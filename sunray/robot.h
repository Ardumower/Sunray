// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"
#include "config.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/CanRobotDriver.h"
#include "src/driver/SerialRobotDriver.h"
#include "src/driver/SimRobotDriver.h"
#include "src/driver/MpuDriver.h"
#include "src/driver/BnoDriver.h"
#include "src/driver/IcmDriver.h"
#include "battery.h"
#include "ble.h"
#include "pinman.h"
#include "bumper.h"
#include "buzzer.h"
#include "sonar.h"
#include "VL53L0X.h"
#include "map.h"   
#include "comm.h"
#include "mqtt.h"
#include "httpserver.h"
#include "StateEstimator.h"
#include "Stats.h"
#include "LineTracker.h"
#include "Storage.h"
#include "src/ublox/ublox.h"
#include "src/skytraq/skytraq.h"
#include "src/lidar/lidar.h"
#ifdef __linux__
  #include <BridgeClient.h>
  #include "src/ntrip/ntripclient.h"
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "PubSubClient.h"
#include "timetable.h"


#define VER "Sunray,1.0.331"

// common types
#include "types.h"

#ifndef __linux__
  #define FILE_CREATE  (O_WRITE | O_CREAT)
#endif

// moved into StateEstimator

extern WiFiEspClient client;
extern PubSubClient mqttClient;
extern bool hasClient;


#ifdef DRV_SERIAL_ROBOT
  extern SerialRobotDriver robotDriver;
  extern SerialMotorDriver motorDriver;
  extern SerialBatteryDriver batteryDriver;
  extern SerialBumperDriver bumperDriver;
  extern SerialStopButtonDriver stopButton;
  extern SerialRainSensorDriver rainDriver;
  extern SerialLiftSensorDriver liftDriver;  
  extern SerialBuzzerDriver buzzerDriver;
  extern RelaisDriver relaisDriver;
#elif DRV_CAN_ROBOT
  extern CanRobotDriver robotDriver;
  extern CanMotorDriver motorDriver;
  extern CanBatteryDriver batteryDriver;
  extern CanBumperDriver bumperDriver;
  extern CanStopButtonDriver stopButton;
  extern CanRainSensorDriver rainDriver;
  extern CanLiftSensorDriver liftDriver;  
  extern CanBuzzerDriver buzzerDriver;
  extern CanRelaisDriver relaisDriver;
#elif DRV_SIM_ROBOT
  extern SimRobotDriver robotDriver;
  extern SimMotorDriver motorDriver;
  extern SimBatteryDriver batteryDriver;
  extern SimBumperDriver bumperDriver;
  extern SimStopButtonDriver stopButton;
  extern SimRainSensorDriver rainDriver;
  extern SimLiftSensorDriver liftDriver;
  extern SimBuzzerDriver buzzerDriver;
  extern RelaisDriver relaisDriver;
#else
  extern AmRobotDriver robotDriver;
  extern AmMotorDriver motorDriver;
  extern AmBatteryDriver batteryDriver;
  extern AmBumperDriver bumperDriver;
  extern AmStopButtonDriver stopButton;
  extern AmRainSensorDriver rainDriver;
  extern AmLiftSensorDriver liftDriver;
  extern AmBuzzerDriver buzzerDriver;
  extern RelaisDriver relaisDriver;
#endif

#ifdef DRV_SIM_ROBOT
  extern SimImuDriver imuDriver;
#elif defined(GPS_LIDAR)
  extern LidarImuDriver imuDriver;
#elif defined(BNO055)
  extern BnoDriver imuDriver;  
#elif defined(ICM20948)
  extern IcmDriver imuDriver;  
#else
  extern MpuDriver imuDriver;
#endif

extern Motor motor;
extern Battery battery;
extern BLEConfig bleConfig;
extern BLEComm ble;
extern Bumper bumper;
extern Buzzer buzzer;
extern LidarBumperDriver lidarBumper;
extern Sonar sonar;
extern VL53L0X tof;
extern PinManager pinMan;
extern Map maps;
extern Comm comm;
extern MqttService mqttService;
extern HttpServer httpServer;
extern StateEstimator stateEstimator;
extern Stats stats;
extern LineTracker lineTracker;
extern Storage storage;
extern TimeTable timetable;
#ifdef DRV_SIM_ROBOT
  extern SimGpsDriver gps;
#elif GPS_LIDAR
  extern LidarGpsDriver gps;
#elif GPS_SKYTRAQ
  extern SKYTRAQ gps;
#else
  extern UBLOX gps;
#endif

int freeMemory();
void start();
void run();
void setOperation(OperationType op, bool allowRepeat = false);
void triggerObstacle();
void sensorTest();
void updateStateOpText();
void detectSensorMalfunction();
bool detectLift();
bool detectObstacle();
bool detectObstacleRotation();



#endif
