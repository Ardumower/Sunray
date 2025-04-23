// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../StateEstimator.h"
#include "../../helper.h"
#include "../../events.h"

ChargeOp chargeOp;
ErrorOp errorOp;
DockOp dockOp;
IdleOp idleOp;
MowOp mowOp;
EscapeReverseOp escapeReverseOp;
EscapeForwardOp escapeForwardOp;
KidnapWaitOp kidnapWaitOp;
GpsWaitFixOp gpsWaitFixOp;
GpsWaitFloatOp gpsWaitFloatOp;
GpsRebootRecoveryOp gpsRebootRecoveryOp;
ImuCalibrationOp imuCalibrationOp;
RelocalizationOp relocalizationOp;

// active op
Op *activeOp = &idleOp;

// new active operation to call
Op *newActiveOp = NULL;



Op::Op(){
    initiatedByOperator = false;
    previousOp = NULL;
    nextOp = NULL;
    shouldStop = false;
}

String Op::name(){
    return "Op";
}


void Op::changeOp(Op &anOp, bool returnBackOnExit){    
    if (&anOp == NULL){
        CONSOLE.println("==> ERROR Op::changeOp: invalid op=NULL!");        
    }
    if (&anOp == activeOp) return;
    newActiveOp = &anOp;

    if (returnBackOnExit) {
      anOp.nextOp = this;
    }
    anOp.previousOp = this;      

    shouldStop = true;              
}

void Op::setInitiatedByOperator(bool flag){
  initiatedByOperator = flag;
}    
    

void Op::changeOperationTypeByOperator(OperationType op){
  CONSOLE.println("changeOperationType");
  if (activeOp == NULL){
    CONSOLE.println("ERROR Op::changeOperationType - activeOp=NULL");
    return;
  }  
  switch (op){
    case OP_IDLE:      
      Logger.event(EVT_USER_STOP);
      activeOp->changeOp(idleOp, false);
      idleOp.setInitiatedByOperator(true);
      break;
    case OP_DOCK:
      Logger.event(EVT_USER_DOCK);
      activeOp->changeOp(dockOp, false);
      dockOp.setInitiatedByOperator(true);
      break;
    case OP_MOW:      
      Logger.event(EVT_USER_START);
      activeOp->changeOp(mowOp, false);
      mowOp.setInitiatedByOperator(true);
      break;
    case OP_CHARGE:
      Logger.event(EVT_CHARGER_CONNECTED);      
      activeOp->changeOp(chargeOp, false);
      chargeOp.setInitiatedByOperator(true);
      break;
    case OP_ERROR:            
      activeOp->changeOp(errorOp, false);
      errorOp.setInitiatedByOperator(true);
      break;
  }
}


OperationType Op::getGoalOperationType(){
    Op *goalOp = getGoalOp();
    if (goalOp == &idleOp) return OP_IDLE;
    if (goalOp == &dockOp) return OP_DOCK;
    if (goalOp == &mowOp) return OP_MOW;
    if (goalOp == &chargeOp) return OP_CHARGE;
    if (goalOp == &errorOp) return OP_ERROR;
    CONSOLE.println("ERROR: Op::getGoalOperationType: invalid goal op!");
    return OP_ERROR;
}



Op* Op::getGoalOp(){
    Op *goalOp = this;
    int chainCounter = 0;    
    while (goalOp->nextOp != NULL) {
      goalOp = goalOp->nextOp;
      if (goalOp == this) break; // avoid infinite loops if same op as this
      chainCounter++;
      if (chainCounter > 10){
          CONSOLE.println("ERROR Op::getGoalOp: invalid op chain (infinity)");
          break;
      }      
    }
    return goalOp; 
}

String Op::getOpChain(){
    String opChain = name();
    opChain += "(initiatedByOperator ";
    opChain += initiatedByOperator;
    opChain += ")";
    Op *goalOp = this;
    int chainCounter = 0;
    while (goalOp->nextOp != NULL) {
        goalOp = goalOp->nextOp;
        if (goalOp == this) break; // avoid infinite loops if same op as this
        opChain += "->";
        opChain += goalOp->name();
        chainCounter++;
        if (chainCounter > 10){
          CONSOLE.println("ERROR Op::getOpChain: invalid op chain (infinity)");
          break;
        }
    }
    return opChain; 
}
    

void Op::begin(){
}


void Op::end(){

}

void Op::checkStop(){
    if (shouldStop){
      if (newActiveOp == NULL){
        CONSOLE.println("ERROR Op::checkStop: invalid newActiveOp=NULL");
        return;
      }
      end();      
      activeOp = newActiveOp;
      newActiveOp = NULL;
      activeOp->startTime = millis();
      CONSOLE.print("==> changeOp:");
      activeOp->OpChain =  activeOp->getOpChain();      
      CONSOLE.println(activeOp->OpChain);
      activeOp->shouldStop = false;
      activeOp->begin();
    }
}

float Op::getDockDistance(){
    float dockX = 0;
    float dockY = 0;
    float dockDelta = 0;
    maps.getDockingPos(dockX, dockY, dockDelta);
    float dist_dock = distance(dockX, dockY, stateX, stateY);

    return dist_dock;
}

void Op::run(){
}

void Op::onKidnapped(bool state){
}

void Op::onGpsNoSignal(){    
}

void Op::onGpsFixTimeout(){
}

void Op::onRainTriggered(){
}

void Op::onTempOutOfRangeTriggered(){
}

void Op::onLiftTriggered(){
}

void Op::onOdometryError(){
}


void Op::onMotorOverload(){
}
    
void Op::onMotorError(){    
}


void Op::onObstacle(){
}

void Op::onObstacleRotation(){
}

void Op::onTargetReached(){
}
 
void Op::onChargerDisconnected(){
}

void Op::onBadChargingContactDetected(){
}

void Op::onChargingCompleted(){    
}

void Op::onNoFurtherWaypoints(){    
}

void Op::onImuCalibration(){
    changeOp(imuCalibrationOp, true);
}

void Op::onRelocalization(){
    changeOp(relocalizationOp, true);
}

void Op::onChargerConnected(){            
}

void Op::onBatteryUndervoltage(){
    stateSensor = SENS_BAT_UNDERVOLTAGE;
    changeOp(idleOp);
    //buzzer.sound(SND_OVERCURRENT, true);        
}

void Op::onBatteryLowShouldDock(){    
}

void Op::onTimetableStartMowing(){    
}

void Op::onTimetableStopMowing(){    
}

void Op::onImuTilt(){
}

void Op::onImuError(){
}
