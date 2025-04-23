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
#include "../../events.h"


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
    motor.setReleaseBrakesWhenZero(false);
    motor.setLinearAngularSpeed(0,0);      
    if (((previousOp != &escapeReverseOp) && (previousOp != &escapeForwardOp)) || (DISABLE_MOW_MOTOR_AT_OBSTACLE))  motor.setMowState(false);              
    battery.setIsDocked(false);                
    timetable.setMowingCompletedInCurrentTimeFrame(false);                

    // plan route to next target point 

    dockOp.dockReasonRainTriggered = false;    

    if (((initiatedByOperator) && (previousOp == &idleOp)) || (lastMapRoutingFailed))  maps.clearObstacles();

    if (maps.startMowing(stateX, stateY)){
        if (maps.nextPoint(true, stateX, stateY)) {
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
            Logger.event(EVT_ERROR_NO_MAP_ROUTE_GIVEUP);
            changeOp(errorOp);      
        } else {    
            Logger.event(EVT_ERROR_NO_MAP_ROUTE);
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
    
    if (timetable.shouldAutostopNow()){
        if (DOCKING_STATION){
            CONSOLE.println("TIMETABLE - DOCKING");
            dockOp.setInitiatedByOperator(false);
            changeOp(dockOp);
        } else {
            CONSOLE.println("TIMETABLE - IDLE");
            changeOp(idleOp);
        }
    }
}

void MowOp::onRainTriggered(){
    if (DOCKING_STATION){
        CONSOLE.println("RAIN TRIGGERED");
        Logger.event(EVT_RAIN_DOCKING);
        stateSensor = SENS_RAIN;
        dockOp.dockReasonRainTriggered = true;
        #ifdef DRV_SIM_ROBOT
            dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 3; // try again after 3 minutes 
        #else
            dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60; // try again after one hour 
        #endif
        dockOp.setInitiatedByOperator(false);
        changeOp(dockOp);              
    }
}

void MowOp::onTempOutOfRangeTriggered(){
    if (DOCKING_STATION){
        CONSOLE.println("TEMP OUT-OF-RANGE TRIGGERED");
        Logger.event(EVT_TEMPERATURE_OUT_OF_RANGE_DOCK);
        stateSensor = SENS_TEMP_OUT_OF_RANGE;
        dockOp.dockReasonRainTriggered = true;
        dockOp.dockReasonRainAutoStartTime = millis() + 60000 * 60; // try again after one hour      
        dockOp.setInitiatedByOperator(false);
        changeOp(dockOp);              
    }
}

void MowOp::onBatteryLowShouldDock(){    
    CONSOLE.println("BATTERY LOW TRIGGERED - DOCKING");
    Logger.event(EVT_BATTERY_LOW_DOCK);
    dockOp.setInitiatedByOperator(false);
    changeOp(dockOp);
}

void MowOp::onTimetableStopMowing(){        
}

void MowOp::onTimetableStartMowing(){        
}

void MowOp::onObstacle(){
    if ((!DOCK_DETECT_OBSTACLE_IN_DOCK) && (maps.isBetweenLastAndNextToLastDockPoint())) {
      //CONSOLE.println("triggerObstacle: ignoring, because in dock");      
      return;
    }
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
        Logger.event(EVT_ERROR_ODOMETRY);
        changeOp(errorOp);
    }
}
    
void MowOp::onMotorOverload(){
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 20000){
        CONSOLE.println("error: motor overload!");    
        stateSensor = SENS_OVERLOAD;
        Logger.event(EVT_ERROR_MOTOR_OVERLOAD);
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
            CONSOLE.print("MowOp::onMotorError motorErrorCounter=");       
            CONSOLE.println(motorErrorCounter);
            if (maps.wayMode != WAY_DOCK){
                if (motorErrorCounter < FAULT_MAX_SUCCESSIVE_ALLOWED_COUNT){                     
                    //stateSensor = SENS_MOTOR_ERROR;
                    Logger.event(EVT_ERROR_MOTOR_ERROR);            
                    changeOp(escapeReverseOp, true);     // trigger obstacle avoidance 
                    return;
                }
            }
            // obstacle avoidance failed with too many motor errors (it was probably not a molehole situation)
            CONSOLE.println("error: motor error - giving up!");
            motorErrorCounter = 0;
            stateSensor = SENS_MOTOR_ERROR;
            Logger.event(EVT_ERROR_MOTOR_ERROR_GIVEUP);
            changeOp(errorOp);
            return;      
        }  
    } else {
        CONSOLE.println("no obstacle avoidance activated on motor errors, giving up");    
        stateSensor = SENS_MOTOR_ERROR;
        Logger.event(EVT_ERROR_MOTOR_ERROR_GIVEUP);
        changeOp(errorOp);        
        return;
    }
}

void MowOp::onTargetReached(){
    if (maps.wayMode == WAY_MOW){    
        maps.clearObstacles(); // clear obstacles if target reached
        motorErrorCounter = 0; // reset motor error counter if target reached
        stateSensor = SENS_NONE; // clear last triggered sensor
    }
}


void MowOp::onGpsFixTimeout(){
    // no gps solution
    if (REQUIRE_VALID_GPS){
#ifdef UNDOCK_IGNORE_GPS_DISTANCE
        if (!maps.isUndocking() || getDockDistance() > UNDOCK_IGNORE_GPS_DISTANCE){
#else
        if (!maps.isUndocking()){
#endif
            stateSensor = SENS_GPS_FIX_TIMEOUT;            
            changeOp(gpsWaitFixOp, true);
        }
    }
}

void MowOp::onGpsNoSignal(){
    if (REQUIRE_VALID_GPS){
#ifdef UNDOCK_IGNORE_GPS_DISTANCE
        if (!maps.isUndocking() || getDockDistance() > UNDOCK_IGNORE_GPS_DISTANCE){
#else
        if (!maps.isUndocking()){
#endif
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
    Logger.event(EVT_MOWING_COMPLETED);
    timetable.setMowingCompletedInCurrentTimeFrame(true);
    if (!finishAndRestart){             
        if (DOCKING_STATION && dockAfterFinish){
            dockOp.setInitiatedByOperator(false);
            changeOp(dockOp);               
        } else {
            idleOp.setInitiatedByOperator(false);
            changeOp(idleOp); 
        }
    }
}

void MowOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    Logger.event(EVT_ROBOT_TILTED);
    changeOp(errorOp);
}

void MowOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    Logger.event(EVT_ERROR_IMU_TIMEOUT);
    changeOp(errorOp);
}

