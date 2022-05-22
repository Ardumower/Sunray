// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsRebootRecoveryOp::name(){
    return "GpsRebootRecovery";
}

void GpsRebootRecoveryOp::begin(){
    // try GPS reboot after 5 minutes
    if (GPS_REBOOT_RECOVERY){
        gps.reboot();  // try to recover from false GPS fix
    }
    retryOperationTime = millis() + 30000; // wait 30 secs after reboot, then try another map routing
}


void GpsRebootRecoveryOp::end(){
}


void GpsRebootRecoveryOp::run(){
    battery.resetIdle();
    if (millis() > retryOperationTime){
        // restart current operation from new position (restart path planning)
        CONSOLE.println("restarting operation (retryOperationTime)");
        retryOperationTime = 0;
        motor.stopImmediately(true);
        changeOp(*nextOp);    // restart current operation      
    }
}

