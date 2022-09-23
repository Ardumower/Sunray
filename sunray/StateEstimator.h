// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>


extern float stateX;  // position-east (m)
extern float stateY;  // position-north (m)
extern float stateDelta;  // direction (rad)
extern float stateRoll;
extern float statePitch;
extern float stateDeltaGPS;
extern float stateDeltaIMU;
extern float stateGroundSpeed; // m/s
extern float lateralError; // lateral error

extern float stateDeltaLast;
extern float stateDeltaSpeed;
extern float stateDeltaSpeedLP;
extern float stateDeltaSpeedIMU;
extern float stateDeltaSpeedWheels;
extern float diffIMUWheelYawSpeed;
extern float diffIMUWheelYawSpeedLP;

extern bool gpsJump;

extern bool imuIsCalibrating;
extern unsigned long imuDataTimeout;
extern float lastIMUYaw; 


bool startIMU(bool forceIMU);
void readIMU();
void computeRobotState();
void resetImuTimeout();


#endif

