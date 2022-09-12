// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../LineTracker.h"
#include "../../map.h"

String KidnapWaitOp::name(){
  return "KidnapWait";
}

void KidnapWaitOp::begin(){    
  stateSensor = SENS_KIDNAPPED;
  recoverGpsTime = millis() + 30000;
  recoverGpsCounter = 0;
  recoverGpsTimeEarliestNext = millis();
}


void KidnapWaitOp::end(){
}

void KidnapWaitOp::onKidnapped(bool state){
  if (!state) {
    changeOp(*nextOp);
  }
}

void KidnapWaitOp::onGpsNoSignal(){
    if (!maps.isUndocking()){
        stateSensor = SENS_GPS_INVALID;
        changeOp(gpsWaitFloatOp, true);
    }
}


void KidnapWaitOp::run(){  
  trackLine(false);       
  battery.resetIdle();

  if (millis() > recoverGpsTime){
    CONSOLE.println("KIDNAP_DETECT");
    recoverGpsTime = millis() + 30000;
    recoverGpsCounter++;
    if (recoverGpsCounter == 3){          
      CONSOLE.println("error: kidnapped!");
      stateSensor = SENS_KIDNAPPED;
      changeOp(errorOp);
      return;
    }   
    if (GPS_REBOOT_RECOVERY && recoverGpsTimeEarliestNext < millis()){           
      gps.reboot();   // try to recover from false GPS fix     
      // give the GPS tracker a fair chance to lock in
      recoverGpsTimeEarliestNext = millis() + 5*60000;
    }
  }
}


