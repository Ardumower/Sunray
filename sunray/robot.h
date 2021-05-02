// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"
#include "config.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/SerialRobotDriver.h"
#include "battery.h"
#include "ble.h"
#include "pinman.h"
#include "buzzer.h"
#include "sonar.h"
#include "VL53L0X.h"
#include "map.h"   
#include "src/ublox/ublox.h"
#ifdef __linux__
  #include <BridgeClient.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "PubSubClient.h"


#define VER "Ardumower Sunray,1.0.183"

enum OperationType {
      OP_IDLE,      
      OP_MOW,            
      OP_CHARGE,      
      OP_ERROR,    
      OP_DOCK,            
};    

enum Sensor {
      SENS_NONE,
      SENS_BAT_UNDERVOLTAGE,            
      SENS_OBSTACLE,      
      SENS_GPS_FIX_TIMEOUT,
      SENS_IMU_TIMEOUT,
      SENS_IMU_TILT,
      SENS_KIDNAPPED,
      SENS_OVERLOAD,
      SENS_MOTOR_ERROR,
      SENS_GPS_INVALID,
      SENS_ODOMETRY_ERROR,
      SENS_MAP_NO_ROUTE,
      SENS_MEM_OVERFLOW,
      SENS_BUMPER,
      SENS_SONAR,
      SENS_LIFT,
      SENS_RAIN,
      SENS_STOP_BUTTON,
};

#ifndef __linux__
  #define FILE_CREATE  (O_WRITE | O_CREAT)
#endif

extern OperationType stateOp; // operation
extern Sensor stateSensor; // last triggered sensor
extern float stateX;  // position-east (m)
extern float stateY;  // position-north (m)
extern float stateDelta;  // direction (rad)
extern String stateOpText;  // current operation as text
extern String gpsSolText; // current gps solution as text
extern int stateButton;  // button state

extern float setSpeed; // linear speed (m/s)
extern int fixTimeout;
extern bool finishAndRestart; // auto-restart when mowing finished?
extern bool absolutePosSource;
extern double absolutePosSourceLon;
extern double absolutePosSourceLat;

extern unsigned long statIdleDuration; // seconds
extern unsigned long statChargeDuration; // seconds
extern unsigned long statMowDuration ; // seconds
extern unsigned long statMowDurationInvalid ; // seconds
extern unsigned long statMowDurationFloat ; // seconds
extern unsigned long statMowDurationFix ; // seconds
extern unsigned long statMowFloatToFixRecoveries ; // counter
extern unsigned long statMowInvalidRecoveries ; // counter
extern unsigned long statImuRecoveries ; // counter
extern unsigned long statMowObstacles ; // counter
extern unsigned long statGPSJumps ; // counter
extern unsigned long statMowGPSMotionTimeoutCounter;
extern unsigned long statMowBumperCounter; 
extern unsigned long statMowSonarCounter;
extern unsigned long statMowLiftCounter;
extern float statMowMaxDgpsAge ; // seconds
extern float statMowDistanceTraveled ; // meter
extern float statTempMin;
extern float statTempMax;

extern unsigned long lastFixTime;
extern float stateGroundSpeed; // m/s

extern WiFiEspClient client;
extern WiFiEspServer server;
extern PubSubClient mqttClient;
extern bool hasClient;

extern unsigned long controlLoops;
extern bool imuIsCalibrating;
extern bool wifiFound;

#ifdef DRV_SERIAL_ROBOT
  extern SerialRobotDriver robotDriver;
  extern SerialMotorDriver motorDriver;
  extern SerialBatteryDriver batteryDriver;
  extern SerialBumperDriver bumper;
  extern SerialStopButtonDriver stopButton;
#else
  extern AmRobotDriver robotDriver;
  extern AmMotorDriver motorDriver;
  extern AmBatteryDriver batteryDriver;
  extern AmBumperDriver bumper;
  extern AmStopButtonDriver stopButton;
#endif

extern Motor motor;
extern Battery battery;
extern BLEConfig bleConfig;
extern Buzzer buzzer;
extern Sonar sonar;
extern VL53L0X tof;
extern PinManager pinMan;
extern Map maps;
extern UBLOX gps;

int freeMemory();
void start();
void run();
void setOperation(OperationType op, bool allowRepeat = false, bool initiatedbyOperator = false);
void triggerObstacle();
void sensorTest();
void updateStateOpText();

#endif
