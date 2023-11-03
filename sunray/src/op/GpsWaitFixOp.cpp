// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsWaitFixOp::name(){
    return "GpsWaitFix";
}

void GpsWaitFixOp::begin(){
    CONSOLE.println("WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;
    //setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);          
    
    //linear = 0;
    //angular = 0;      
    //mow = false;
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);     

    // store entry time to trigger GPS reboot
    gpsReinitTime = millis() + 120000;
}


void GpsWaitFixOp::end(){
}

void GpsWaitFixOp::run(){
    battery.resetIdle();
    if (gps.solution == SOL_FIXED){
        changeOp(*nextOp);
    }     

    // reboot GPS every 120s if no change
    if(millis() > gpsReinitTime){
        Console.print("GPSWaitFloat: Rebooting GPS...");
        gps.reboot();
        gpsReinitTime = millis() + 120000;
    }
}


