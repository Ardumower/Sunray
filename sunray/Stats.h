// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STATS_H
#define STATS_H


#include <Arduino.h>


extern unsigned long statIdleDuration; // seconds
extern unsigned long statChargeDuration; // seconds
extern unsigned long statMowDuration ; // seconds
extern unsigned long statMowDurationInvalid ; // seconds
extern unsigned long statMowDurationFloat ; // seconds
extern unsigned long statMowDurationFix ; // seconds
extern unsigned long statMowDurationMotorRecovery ; // seconds
extern unsigned long statMowFloatToFixRecoveries ; // counter
extern unsigned long statMowInvalidRecoveries ; // counter
extern unsigned long statImuRecoveries ; // counter
extern unsigned long statMowObstacles ; // counter
extern unsigned long statGPSJumps ; // counter
extern unsigned long statMowGPSMotionTimeoutCounter;
extern unsigned long statMowGPSNoSpeedCounter;
extern unsigned long statMowObstacleDetectionRotationCounter;
extern unsigned long statMowBumperCounter; 
extern unsigned long statMowSonarCounter;
extern unsigned long statMowLiftCounter;
extern unsigned long statMowImuNoRotationSpeedCounter;
extern unsigned long statMowDiffIMUWheelYawSpeedCounter;
extern unsigned long statMowToFCounter;
extern unsigned long statMowRotationTimeoutCounter;
extern float statMowMaxDgpsAge ; // seconds
extern float statMowDistanceTraveled ; // meter
extern float statTempMin;
extern float statTempMax;

void calcStats();

#endif


