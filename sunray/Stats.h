// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATS_H
#define STATS_H


#include <Arduino.h>
#include "gps.h"


class Stats {
public:
  // counters/timers
  unsigned long statIdleDuration = 0; // seconds
  unsigned long statChargeDuration = 0; // seconds
  unsigned long statMowDuration = 0; // seconds
  unsigned long statMowDurationInvalid = 0; // seconds
  unsigned long statMowDurationFloat = 0; // seconds
  unsigned long statMowDurationFix = 0; // seconds
  unsigned long statMowDurationMotorRecovery = 0; // seconds
  unsigned long statMowFloatToFixRecoveries = 0; // counter
  unsigned long statMowInvalidRecoveries = 0; // counter
  unsigned long statImuRecoveries = 0; // counter
  unsigned long statMowObstacles = 0; // counter
  unsigned long statGPSJumps = 0; // counter
  unsigned long statMowGPSMotionTimeoutCounter = 0;
  unsigned long statMowGPSNoSpeedCounter = 0;
  unsigned long statMowObstacleDetectionRotationCounter = 0;
  unsigned long statMowBumperCounter = 0; 
  unsigned long statMowSonarCounter = 0;
  unsigned long statMowLiftCounter = 0;
  unsigned long statMowImuNoRotationSpeedCounter = 0;
  unsigned long statMowDiffIMUWheelYawSpeedCounter = 0;
  unsigned long statMowToFCounter = 0;
  unsigned long statMowRotationTimeoutCounter = 0;
  float statMowMaxDgpsAge = 0; // seconds
  float statMowDistanceTraveled = 0; // meter
  float statTempMin = 9999;
  float statTempMax = -9999;

  void calc();

private:
  unsigned long nextStatTime = 0;
  SolType lastSolution = SOL_INVALID;
};


#endif
