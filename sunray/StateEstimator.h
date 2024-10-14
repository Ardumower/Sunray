// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>

// localization modes
enum LocalizationMode {
      LOC_IMU_ODO_ONLY, // IMU/odometry-only localization 
      LOC_GPS,          // RTK localization
      LOC_APRIL_TAG,    // vision (camera) localization (april-tag)       
      LOC_GUIDANCE_SHEET, // sideways guidance sheets localization (one dimension)
      LOC_REFLECTOR_TAG,    // LiDAR localization (reflector-tag)       
};    

extern LocalizationMode stateLocalizationMode;


extern float stateX;  // position-east (m)
extern float stateY;  // position-north (m)
extern float stateDelta;  // direction (rad)
extern float stateRoll;
extern float statePitch;
extern float stateDeltaGPS;
extern float stateDeltaIMU;

extern bool stateAprilTagFound; // found april-tag in camera stream?
extern float stateXAprilTag; // camera-position in april-tag frame
extern float stateYAprilTag;  
extern float stateDeltaAprilTag; 

extern bool stateReflectorTagFound; // found reflector-tag in camera stream?
extern bool stateReflectorTagOutsideFound; // found outside reflector-tag in camera stream?
extern bool stateReflectorUndockCompleted; // completed undocking with reflector?
extern float stateXReflectorTag; // camera-position in reflector-tag frame
extern float stateYReflectorTag;  
extern float stateDeltaReflectorTag; 

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

