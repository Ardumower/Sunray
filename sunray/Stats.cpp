// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "Stats.h"
#include "robot.h"
#include "motor.h"
#include <Arduino.h>



// calculate statistics
void Stats::calc(){
  if (millis() >= nextStatTime){
    nextStatTime = millis() + 1000;
    switch (stateEstimator.stateOp){
      case OP_IDLE:
        statIdleDuration++;
        break;
      case OP_MOW:      
        statMowDuration++;
        if (motor.motorRecoveryState) statMowDurationMotorRecovery++;
        if (gps.solution == SOL_FIXED) statMowDurationFix++;
          else if (gps.solution == SOL_FLOAT) statMowDurationFloat++;   
          else if (gps.solution == SOL_INVALID) statMowDurationInvalid++;
        if (gps.solution != lastSolution){      
          if ((lastSolution == SOL_FLOAT) && (gps.solution == SOL_FIXED)) statMowFloatToFixRecoveries++;
          if (lastSolution == SOL_INVALID) statMowInvalidRecoveries++;
          lastSolution = gps.solution;
        } 
        statMowMaxDgpsAge = max(statMowMaxDgpsAge, (millis() - gps.dgpsAge)/1000.0);        
        break;
      case OP_CHARGE:
        statChargeDuration++;
        break;
    }     
  }   
}
