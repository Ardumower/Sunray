// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef ROBOT_H
#define ROBOT_H


#include "motor.h"
#include "battery.h"
#include "ble.h"
#include "pinman.h"
#include "buzzer.h"
#include "sonar.h"
#include "map.h"   
#include "ublox.h"
#include "WiFiEsp.h"


#define VER "Ardumower Sunray,1.0.70"

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
};

extern OperationType stateOp; // operation
extern Sensor stateSensor; // last triggered sensor
extern float stateX;  // position-east (m)
extern float stateY;  // position-north (m)
extern float stateDelta;  // direction (rad)

extern float setSpeed; // linear speed (m/s)
extern int fixTimeout;
extern bool finishAndRestart; // auto-restart when mowing finished?
extern bool absolutePosSource;
extern double absolutePosSourceLon;
extern double absolutePosSourceLat;

extern unsigned long statIdleDuration; // seconds
extern unsigned long statChargeDuration; // seconds
extern unsigned long statMowDuration ; // seconds
extern unsigned long statMowDurationFloat ; // seconds
extern unsigned long statMowDurationFix ; // seconds
extern unsigned long statMowFloatToFixRecoveries ; // counter
extern unsigned long statImuRecoveries ; // counter
extern float statMowMaxDgpsAge ; // seconds
extern float statMowDistanceTraveled ; // meter
extern float statTempMin;
extern float statTempMax;

extern WiFiEspClient client;
extern WiFiEspServer server;

extern unsigned long controlLoops;
extern bool wifiFound;

extern "C" char* sbrk(int incr);

extern Motor motor;
extern Battery battery;
extern BLEConfig bleConfig;
extern Buzzer buzzer;
extern Sonar sonar;
extern PinManager pinMan;
extern Map maps;
extern UBLOX gps;

extern int freeMemory();
extern void start();
extern void run();
extern void setOperation(OperationType op);


#endif
