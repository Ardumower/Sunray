// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H


#include <Arduino.h>
#include "config.h"


class LineTracker {
public:
  // Stanley controller gains (exposed for tuning)
  float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
  float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
  float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
  float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

  void trackLine(bool runControl);

private:
  bool rotateLeft = false;
  bool rotateRight = false;
  bool angleToTargetFits = false;
  bool langleToTargetFits = false;
  bool targetReached = false;
  float trackerDiffDelta = 0;
  bool stateKidnapped = false;
  bool printmotoroverload = false;
  bool trackerDiffDelta_positive = false;
  float lastLineDist = 0;
};



#endif
