// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../events.h"

String GpsWaitFixOp::name(){
    return "GpsWaitFix";
}

void GpsWaitFixOp::begin(){
    CONSOLE.println("WARN: no gps solution!");
    Logger.event(EVT_GPS_BAD);
    stateSensor = SENS_GPS_INVALID;
    //setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);          
    
    //linear = 0;
    //angular = 0;      
    //mow = false;
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);     
}


void GpsWaitFixOp::end(){
}

void GpsWaitFixOp::run(){
    battery.resetIdle();
    if (gps.solution == SOL_FIXED){
        changeOp(*nextOp);
    }     
}


