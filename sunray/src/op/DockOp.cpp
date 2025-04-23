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

DockOp::DockOp(){
  lastMapRoutingFailed = false;
  mapRoutingFailedCounter = 0;
  dockReasonRainTriggered = false;
  dockReasonRainAutoStartTime = 0;
}


String DockOp::name(){
    return "Dock";
}


void DockOp::begin(){
  if (previousOp == &chargeOp){
    battery.setIsDocked(true);    
    changeOp(chargeOp);    
    return;
  }

  bool error = false;
  bool routingFailed = false;      
  
  motor.setLinearAngularSpeed(0,0);
  motor.setMowState(false);                

  if (((initiatedByOperator) && (previousOp == &idleOp)) || (lastMapRoutingFailed))  maps.clearObstacles();
  
  CONSOLE.print("OP_DOCK");
  CONSOLE.print(" initiatedByOperator=");
  CONSOLE.print(initiatedByOperator);
  CONSOLE.print(" dockReasonRainTriggered=");
  CONSOLE.println(dockReasonRainTriggered);

  // plan route to next target point 

  if (maps.startDocking(stateX, stateY)){       
    if (maps.nextPoint(true, stateX, stateY)) {
      maps.repeatLastMowingPoint();
      lastFixTime = millis();                
      maps.setLastTargetPoint(stateX, stateY);        
      //stateSensor = SENS_NONE;                  
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
      CONSOLE.println("error: too many routing errors!");
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



void DockOp::end(){
}

void DockOp::run(){
    if (!detectObstacle()){
        detectObstacleRotation();                              
    }
    // line tracking
    trackLine(true);       
    detectSensorMalfunction(); 
    battery.resetIdle();
}


void DockOp::onTargetReached(){
    CONSOLE.println("DockOp::onTargetReached");
    if (maps.wayMode == WAY_MOW){
      maps.clearObstacles(); // clear obstacles if target reached
      motorErrorCounter = 0; // reset motor error counter if target reached
      stateSensor = SENS_NONE; // clear last triggered sensor
    }
}


void DockOp::onGpsFixTimeout(){
    if (REQUIRE_VALID_GPS){    
      stateSensor = SENS_GPS_FIX_TIMEOUT;
      changeOp(gpsWaitFixOp, true);
    }
}

void DockOp::onGpsNoSignal(){
    if (REQUIRE_VALID_GPS){   
      stateSensor = SENS_GPS_INVALID;
      changeOp(gpsWaitFloatOp, true);
    }
}

void DockOp::onKidnapped(bool state){
    if (state){
        stateSensor = SENS_KIDNAPPED;      
        motor.setLinearAngularSpeed(0,0, false); 
        motor.setMowState(false);    
        changeOp(kidnapWaitOp, true); 
    }
}


void DockOp::onObstacleRotation(){
    CONSOLE.println("error: rotation error due to obstacle!");    
    statMowObstacles++;   
    stateSensor = SENS_OBSTACLE;
    changeOp(errorOp);    
}

void DockOp::onObstacle(){
    if (battery.chargerConnected()) {
      CONSOLE.println("triggerObstacle: ignoring, because charger connected");      
      return;
    }
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

void DockOp::onChargerConnected(){            
  battery.setIsDocked(true);    
  changeOp(chargeOp);
}


void DockOp::onNoFurtherWaypoints(){
  CONSOLE.println("docking finished!");
  battery.setIsDocked(true);    
  changeOp(idleOp); 
}


