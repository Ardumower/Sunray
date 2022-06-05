// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H


#include <Arduino.h>


extern float stanleyTrackingNormalK;
extern float stanleyTrackingNormalP;
extern float stanleyTrackingSlowK;
extern float stanleyTrackingSlowP;

extern int dockGpsRebootState;          // Svol0: state for gps-reboot at specified docking point by undocking action
extern bool dockGpsRebootDistGpsTrg;    // Svol0: trigger to check solid gps-fix position (no jump)
extern bool blockKidnapByUndocking;     // Svol0: kidnap detection is blocked by undocking without gps

void trackLine(bool runControl);  


#endif
