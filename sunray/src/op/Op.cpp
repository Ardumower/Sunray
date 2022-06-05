// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"


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

// active op
Op *activeOp = &idleOp;


Op::Op(){
    initiatedbyOperator = false;
    previousOp = NULL;
    nextOp = NULL;
    shouldStop = false;
}

String Op::name(){
    return "Op";
}


void Op::changeOp(Op &anOp, bool returnBackOnExit, bool initiatedbyOperatorFlag){    
    if (&anOp == NULL){
        CONSOLE.println("==> ERROR Op::changeOp: invalid op=NULL!");        
    }
    if (&anOp == activeOp) return;
    nextOp = &anOp;

    if (returnBackOnExit) {
      anOp.nextOp = this;
    }
    anOp.previousOp = this;
    anOp.initiatedbyOperator = initiatedbyOperatorFlag;         

    shouldStop = true;              
}


void Op::changeOperationType(OperationType op, bool initiatedbyOperatorFlag){
  if (activeOp == NULL){
    CONSOLE.println("ERROR Op::changeOperationType - activeOp=NULL");
    return;
  }
  switch (op){
    case OP_IDLE:
      activeOp->changeOp(idleOp);
      break;
    case OP_DOCK:
      activeOp->changeOp(dockOp);
      break;
    case OP_MOW:      
      activeOp->changeOp(mowOp);
      break;
    case OP_CHARGE:
      activeOp->changeOp(chargeOp);
      break;
    case OP_ERROR:            
      activeOp->changeOp(errorOp);
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
      if (nextOp == NULL){
        CONSOLE.println("ERROR Op::checkStop: invalid nextOp=NULL");
        return;
      }
      end();      
      activeOp = nextOp;
      nextOp = NULL;
      activeOp->startTime = millis();
      CONSOLE.print("==> changeOp:");
      CONSOLE.println(activeOp->getOpChain());
      activeOp->shouldStop = false;
      activeOp->begin();
    }
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

void Op::onChargingCompleted(){    
}

void Op::onNoFurtherWaypoints(){    
}

void Op::onImuCalibration(){
    changeOp(imuCalibrationOp, true);
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


void Op::onImuTilt(){
}

void Op::onImuError(){
}
