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


void trackLine(bool runControl);  


#endif

