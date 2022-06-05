// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../LineTracker.h"
#include "../../Stats.h"
#include "../../map.h"


MowOp::MowOp(){
    lastMapRoutingFailed = false;
    mapRoutingFailedCounter = 0;
}

String MowOp::name(){
    return "Mow";
}

void MowOp::begin(){
    bool error = false;
    bool routingFailed = false;      

    CONSOLE.println("OP_MOW");      
    motor.enableTractionMotors(true); // allow traction motors to operate         
    motor.setLinearAngularSpeed(0,0);      
    motor.setMowState(false);                

    // plan route to next target point 

    //dockingInitiatedByOperator = false;
    //dockReasonRainTriggered = false;
    if ((initiatedbyOperator) || (lastMapRoutingFailed)) maps.clearObstacles();
    if (maps.startMowing(stateX, stateY)){
        if (maps.nextPoint(true)) {
            lastFixTime = millis();                
            maps.setLastTargetPoint(stateX, stateY);        
            //stateSensor = SENS_NONE;
            motor.setMowState(true);                
        } else {
            error = true;
            CONSOLE.println("error: no waypoints!");
            //op = stateOp;                
        }
    } else error = true;

    if (error){
        stateSensor = SENS_MAP_NO_ROUTE;
        //op = OP_ERROR;
        routingFailed = true;
        motor.setMowState(false);
    }

    if (routingFailed){
        lastMapRoutingFailed = true; 
        mapRoutingFailedCounter++;    
        if (mapRoutingFailedCounter > 60){
            CONSOLE.println("error: too many map routing errors!");
            stateSensor = SENS_MAP_NO_ROUTE;
            changeOp(errorOp);      
        } else {    
        changeOp(gpsRebootRecoveryOp, true);
        }
    } else {
        lastMapRoutingFailed = false;
        mapRoutingFailedCounter = 0;
    }
}


void MowOp::end(){
}

void MowOp::run(){
    if (!detectObstacle()){
        detectObstacleRotation();                              
    }        
    // line tracking
    trackLine(true); 
    detectSensorMalfunction();    
    battery.resetIdle();
}

void MowOp::onRainTriggered(){
    if (DOCKING_STATION){
        stateSensor = SENS_RAIN;
        dockOp.dockReasonRainTriggered = true;
        changeOp(dockOp);              
    }
}


void MowOp::onBatteryLowShouldDock(){    
    changeOp(dockOp);
}


void MowOp::onObstacle(){
    CONSOLE.println("triggerObstacle");      
    statMowObstacles++;      
    if (maps.isDocking()) {    
        if (maps.retryDocking(stateX, stateY)) {
            changeOp(escapeReverseOp, true);                      
            return;
        }
    } 
    if ((OBSTACLE_AVOIDANCE) && (maps.wayMode != WAY_DOCK)){    
        changeOp(escapeReverseOp, true);      
    } else {     
        stateSensor = SENS_OBSTACLE;
        CONSOLE.println("error: obstacle!");            
        changeOp(errorOp);                
    }
}
    
void MowOp::onObstacleRotation(){
    CONSOLE.println("triggerObstacleRotation");    
    statMowObstacles++;   
    if ((OBSTACLE_AVOIDANCE) && (maps.wayMode != WAY_DOCK)){    
        if (FREEWHEEL_IS_AT_BACKSIDE){    
            changeOp(escapeForwardOp, true);      
        } else {
            changeOp(escapeReverseOp, true);
        }
    } else { 
        stateSensor = SENS_OBSTACLE;
        CONSOLE.println("error: obstacle!");            
        changeOp(errorOp);
    }
}


void MowOp::onOdometryError(){
    if (ENABLE_ODOMETRY_ERROR_DETECTION){
        CONSOLE.println("error: odometry error!");    
        stateSensor = SENS_ODOMETRY_ERROR;
        changeOp(errorOp);
    }
}
    
void MowOp::onMotorOverload(){
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 20000){
        CONSOLE.println("error: motor overload!");    
        stateSensor = SENS_OVERLOAD;
        changeOp(errorOp);
        return;
    }
  }
}
    
void MowOp::onMotorError(){
    if (ENABLE_FAULT_OBSTACLE_AVOIDANCE){
        if (motor.motorError){
            // this is the molehole situation: motor error will permanently trigger on molehole => we try obstacle avoidance (molehole avoidance strategy)
            motor.motorError = false; // reset motor error flag
            motorErrorCounter++;       
            if (maps.wayMode != WAY_DOCK){
                if (motorErrorCounter < 5){ 
                    //stateSensor = SENS_MOTOR_ERROR;
                    changeOp(escapeReverseOp, true);     // trigger obstacle avoidance 
                    return;
                }
            }
            // obstacle avoidance failed with too many motor errors (it was probably not a molehole situation)
            CONSOLE.println("error: motor error!");
            stateSensor = SENS_MOTOR_ERROR;
            changeOp(errorOp);
            return;      
        }  
    }
}

void MowOp::onTargetReached(){
    maps.clearObstacles(); // clear obstacles if target reached
    motorErrorCounter = 0; // reset motor error counter if target reached
    stateSensor = SENS_NONE; // clear last triggered sensor
}


void MowOp::onGpsFixTimeout(){
    // no gps solution
    if (REQUIRE_VALID_GPS){
        if (!maps.isUndocking()){
            stateSensor = SENS_GPS_FIX_TIMEOUT;
            changeOp(gpsWaitFixOp, true);
        }
    }
}

void MowOp::onGpsNoSignal(){
    if (REQUIRE_VALID_GPS){
        if (!maps.isUndocking()){
            stateSensor = SENS_GPS_INVALID;
            changeOp(gpsWaitFloatOp, true);
        }
    }
}

void MowOp::onKidnapped(bool state){
    if (state){
        stateSensor = SENS_KIDNAPPED;      
        motor.setLinearAngularSpeed(0,0, false); 
        motor.setMowState(false);    
        changeOp(kidnapWaitOp, true); 
    }
}

void MowOp::onNoFurtherWaypoints(){
    CONSOLE.println("mowing finished!");
    if (!finishAndRestart){             
        if (DOCKING_STATION){
            changeOp(dockOp);               
        } else {
            changeOp(idleOp); 
        }
    }
}

void MowOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void MowOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}

