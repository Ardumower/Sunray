// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"


String ChargeOp::name(){
    return "Charge";
}


void ChargeOp::begin(){
    CONSOLE.print("OP_CHARGE");
    CONSOLE.print(" dockOp.initiatedByOperator=");
    CONSOLE.print(dockOp.initiatedByOperator);
    CONSOLE.print(" dockReasonRainTriggered=");
    CONSOLE.println(dockOp.dockReasonRainTriggered);

    //motor.stopImmediately(true); // do not use PID to get to stop 
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);     
    //motor.enableTractionMotors(false); // keep traction motors off (motor drivers tend to generate some incorrect encoder values when stopped while not turning)                 
}


void ChargeOp::end(){
}

void ChargeOp::run(){
    battery.resetIdle();
    if (battery.chargerConnected()){        
        //CONSOLE.println("Op::onChargerConnected");
        maps.setIsDocked(true);               
        // get robot position and yaw from docking pos
        // sensing charging contacts means we are in docking station - we use docking point coordinates to get rid of false fix positions in
        // docking station
        maps.getDockingPos(stateX, stateY, stateDelta);
        // get robot yaw orientation from map 
        //float tempX;
        //float tempY;
        //maps.setRobotStatePosToDockingPos(tempX, tempY, stateDelta);                                            
        if (battery.chargingHasCompleted()){
            if ((DOCKING_STATION) && (!dockOp.initiatedByOperator)) {
                if (maps.mowPointsIdx > 0){  // if mowing not completed yet
                    if ((DOCK_AUTO_START) && (!dockOp.dockReasonRainTriggered)) { // automatic continue mowing allowed?
                        CONSOLE.println("DOCK_AUTO_START: will automatically continue mowing now");
                        changeOp(mowOp); // continue mowing
                    }
                }
            }
        }
    }        
}

void ChargeOp::onChargerDisconnected(){
    motor.enableTractionMotors(true); // allow traction motors to operate                       
    maps.setIsDocked(false);
    changeOp(idleOp);
};

void ChargeOp::onBatteryUndervoltage(){    
    stateSensor = SENS_BAT_UNDERVOLTAGE;
}

