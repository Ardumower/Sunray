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
}

String Op::name(){
    return "Op";
}


void Op::changeOp(Op &anOp, bool returnBackOnExit, bool initiatedbyOperatorFlag){    
    if (&anOp == NULL){
        CONSOLE.println("==> ERROR Op::changeOp: invalid op=NULL!");        
    }
    if (&anOp == activeOp) return;
    end();
    if (returnBackOnExit)
        anOp.nextOp = this;
    anOp.previousOp = this;
    activeOp = &anOp;
    anOp.startTime = millis();
    anOp.initiatedbyOperator = initiatedbyOperatorFlag;
    CONSOLE.print("==> changeOp:");
    CONSOLE.println(activeOp->getOpChain());
    anOp.begin();          
}


void Op::changeOperationType(OperationType op, bool initiatedbyOperatorFlag){
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
    while (goalOp->nextOp != NULL) goalOp = goalOp->nextOp;
    return goalOp; 
}

String Op::getOpChain(){
    String opChain = name();
    Op *goalOp = this;
    while (goalOp->nextOp != NULL) {
        goalOp = goalOp->nextOp;
        opChain += "->";
        opChain += goalOp->name();
    }
    return opChain; 
}
    

void Op::begin(){
}


void Op::end(){

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


