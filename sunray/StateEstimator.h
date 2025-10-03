// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H


#include <Arduino.h>
#include "types.h"

// localization modes
enum LocalizationMode {
      LOC_IMU_ODO_ONLY, // IMU/odometry-only localization 
      LOC_GPS,          // RTK localization
      LOC_APRIL_TAG,    // vision (camera) localization (april-tag)       
      LOC_GUIDANCE_SHEET, // sideways guidance sheets localization (one dimension)
      LOC_REFLECTOR_TAG,    // LiDAR localization (reflector-tag)       
};    

class StateEstimator {
public:
  // public state
  LocalizationMode stateLocalizationMode = LOC_GPS;

  float stateX = 0;            // position-east (m)
  float stateY = 0;            // position-north (m)
  float stateDelta = 0;        // direction (rad)
  float stateRoll = 0;
  float statePitch = 0;
  float stateDeltaGPS = 0;
  float stateDeltaIMU = 0;

  bool stateAprilTagFound = false;
  float stateXAprilTag = 0;    // camera-position in april-tag frame
  float stateYAprilTag = 0;
  float stateDeltaAprilTag = 0;

  bool stateReflectorTagFound = false;
  bool stateReflectorTagOutsideFound = false;
  bool stateReflectorUndockCompleted = false;
  float stateXReflectorTag = 0;  // camera-position in reflector-tag frame
  float stateYReflectorTag = 0;
  float stateDeltaReflectorTag = 0;

  float stateGroundSpeed = 0;  // m/s
  float lateralError = 0;      // lateral error

  float stateDeltaLast = 0;
  float stateDeltaSpeed = 0;
  float stateDeltaSpeedLP = 0;
  float stateDeltaSpeedIMU = 0;
  float stateDeltaSpeedWheels = 0;
  float diffIMUWheelYawSpeed = 0;
  float diffIMUWheelYawSpeedLP = 0;

  bool gpsJump = false;

  bool imuIsCalibrating = false;
  float lastIMUYaw = 0;
  bool absolutePosSource = false;
  double absolutePosSourceLon = 0;
  double absolutePosSourceLat = 0;
  // moved from robot.h
  OperationType stateOp = OP_IDLE; // operation
  Sensor stateSensor = SENS_NONE;  // last triggered sensor
  String stateOpText = "";  // current operation as text
  String gpsSolText = "";   // current gps solution as text
  int stateButton = 0;       // button state
  float stateTemp = 20;      // current temperature
  float setSpeed = 0.1f;     // linear speed (m/s)
  int fixTimeout = 0;
  bool finishAndRestart = false; // auto-restart when mowing finished?
  bool dockAfterFinish = true;   // dock after mowing finished?
  unsigned long linearMotionStartTime = 0;
  unsigned long angularMotionStartTime = 0;
  bool stateInMotionLP = false; // motion LP filter flag
  unsigned long lastFixTime = 0;
  // system-level status moved from robot.h
  unsigned long controlLoops = 0;
  bool wifiFound = false;
  int motorErrorCounter = 0;
  unsigned long imuDataTimeout = 0;

  // API
  void begin();
  bool startIMU(bool forceIMU);
  void readIMU();
  void computeRobotState();
  void resetImuTimeout();

private:
  // internal variables (previously file-local)
  float stateXReflectorTagLast = 0;
  unsigned long stateLeftTicks = 0;
  unsigned long stateRightTicks = 0;
  float lastPosN = 0;
  float lastPosE = 0;
  float lastPosDelta = 0;
  bool resetLastPos = true;
  float rollChange = 0;
  float pitchChange = 0;
  int imuCalibrationSeconds = 0;
  unsigned long nextImuCalibrationSecond = 0;
  unsigned long nextDumpTime = 0;

  // helpers
  void testRelativeLL();
  void dumpImuTilt();
};



#endif
