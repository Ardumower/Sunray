// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"
#include "../../events.h"
#include "../../helper.h"


String ChargeOp::name(){
    return "Charge";
}


void ChargeOp::begin(){
    Logger.event(EVT_CHARGER_CONNECTED);
    nextConsoleDetailsTime = 0;
    retryTouchDock = false;
    betterTouchDock = false;
    CONSOLE.print("OP_CHARGE");
    CONSOLE.print(" dockOp.initiatedByOperator=");
    CONSOLE.print(dockOp.initiatedByOperator);
    CONSOLE.print(" dockReasonRainTriggered=");
    CONSOLE.println(dockOp.dockReasonRainTriggered);

    if (DOCK_RELEASE_BRAKES){
        motor.setReleaseBrakesWhenZero(true);
    }
    //motor.stopImmediately(true); // do not use PID to get to stop 
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);         
    //motor.enableTractionMotors(false); // keep traction motors off (motor drivers tend to generate some incorrect encoder values when stopped while not turning)                 
}


void ChargeOp::end(){
}

void ChargeOp::run(){

    if ((retryTouchDock) || (betterTouchDock)){
        if (millis() > retryTouchDockSpeedTime){                            
            retryTouchDockSpeedTime = millis() + 1000;
            motor.enableTractionMotors(true); // allow traction motors to operate                               
            if (DOCK_FRONT_SIDE) motor.setLinearAngularSpeed(0.05, 0);
                else motor.setLinearAngularSpeed(-0.03, 0);
        }
        if (retryTouchDock){
            if (millis() > retryTouchDockStopTime) {
                motor.setLinearAngularSpeed(0, 0);
                retryTouchDock = false;
                CONSOLE.println("ChargeOp: retryTouchDock failed");
                Logger.event(EVT_DOCK_RECOVERY_GIVEUP);
                motor.enableTractionMotors(true); // allow traction motors to operate                               
                maps.setIsDocked(false);
                changeOp(idleOp);    
            }
        } else if (betterTouchDock){
            if (millis() > betterTouchDockStopTime) {
                CONSOLE.println("ChargeOp: betterTouchDock completed");
                motor.setLinearAngularSpeed(0, 0);            
                betterTouchDock = false;
            }        
        }
    }
    
    battery.resetIdle();
    if (battery.chargerConnected()){        
        //CONSOLE.println("Op::onChargerConnected");
        maps.setIsDocked(true);               
        // get robot position and yaw from docking pos
        // sensing charging contacts means we are in docking station - we use docking point coordinates to get rid of false fix positions in
        // docking station        
        if (true){
            maps.getDockingPos(stateX, stateY, stateDelta);
            if (!DOCK_FRONT_SIDE) stateDelta = scalePI(stateDelta + 3.1415);
        }
        // get robot yaw orientation from map 
        //float tempX;
        //float tempY;
        //maps.setRobotStatePosToDockingPos(tempX, tempY, stateDelta);                                            
        if (battery.chargingHasCompleted()){
            if (millis() > nextConsoleDetailsTime){
                nextConsoleDetailsTime = millis() + 30000;
                CONSOLE.print("ChargeOp: charging completed (DOCKING_STATION=");
                CONSOLE.print(DOCKING_STATION);
                CONSOLE.print(", battery.isDocked=");
                CONSOLE.print(battery.isDocked());
                CONSOLE.print(", dockOp.initiatedByOperator=");
                CONSOLE.print(dockOp.initiatedByOperator);        
                CONSOLE.print(", maps.mowPointsIdx=");
                CONSOLE.print(maps.mowPointsIdx);
                CONSOLE.print(", DOCK_AUTO_START=");
                CONSOLE.print(DOCK_AUTO_START);
                CONSOLE.print(", dockOp.dockReasonRainTriggered=");
                CONSOLE.print(dockOp.dockReasonRainTriggered);
                CONSOLE.print(", dockOp.dockReasonRainAutoStartTime(min remain)=");
                CONSOLE.print( ((int)(dockOp.dockReasonRainAutoStartTime - millis())) / 60000 );                                
                CONSOLE.print(", timetable.mowingCompletedInCurrentTimeFrame=");                
                CONSOLE.print(timetable.mowingCompletedInCurrentTimeFrame);
                CONSOLE.print(", timetable.mowingAllowed=");                
                CONSOLE.print(timetable.mowingAllowed());
                CONSOLE.print(", finishAndRestart=");                
                CONSOLE.print(finishAndRestart);                
                CONSOLE.print(", dockAfterFinish=");
                CONSOLE.print(dockAfterFinish);
                CONSOLE.println(")");
            }
            if (timetable.shouldAutostartNow()){
                CONSOLE.println("DOCK_AUTO_START: will automatically continue mowing now");
                changeOp(mowOp); // continue mowing                                                    
            }
        }
    }        
}

void ChargeOp::onTimetableStopMowing(){        
}

void ChargeOp::onTimetableStartMowing(){        
}

void ChargeOp::onChargerDisconnected(){
    if ((DOCKING_STATION) && (DOCK_RETRY_TOUCH)) {    
        CONSOLE.println("ChargeOp::onChargerDisconnected - retryTouchDock");
        Logger.event(EVT_DOCK_RECOVERY);
        retryTouchDock = true;
        retryTouchDockStopTime = millis() + 5000;
        retryTouchDockSpeedTime = millis();
    } else {
        motor.enableTractionMotors(true); // allow traction motors to operate                               
        maps.setIsDocked(false);
        changeOp(idleOp);    
    }
}

void ChargeOp::onBadChargingContactDetected(){
    if ((DOCKING_STATION) && (DOCK_RETRY_TOUCH)) {    
        CONSOLE.println("ChargeOp::onBadChargingContactDetected - betterTouchDock");
        Logger.event(EVT_DOCK_RECOVERY);
        betterTouchDock = true;
        betterTouchDockStopTime = millis() + 5000;
        retryTouchDockSpeedTime = millis();
    } 
}

void ChargeOp::onChargerConnected(){
    if (retryTouchDock){
        CONSOLE.println("ChargeOp: retryTouchDock succeeded");        
        motor.setLinearAngularSpeed(0, 0);
        retryTouchDock = false;
    }
}

void ChargeOp::onBatteryUndervoltage(){    
    stateSensor = SENS_BAT_UNDERVOLTAGE;
}

void ChargeOp::onRainTriggered(){
    if (DOCKING_STATION){
        dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60; // ensure rain sensor is dry for one hour                       
        //CONSOLE.print("RAIN TRIGGERED dockOp.dockReasonRainAutoStartTime=");
        //CONSOLE.println(dockOp.dockReasonRainAutoStartTime);
    }
}
