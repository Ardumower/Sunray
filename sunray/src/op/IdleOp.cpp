// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String IdleOp::name(){
    return "Idle";
}

void IdleOp::begin(){
    CONSOLE.println("OP_IDLE");          
    motor.setLinearAngularSpeed(0,0);
    motor.setMowState(false);
    maps.setIsDocked(false);
}


void IdleOp::end(){

}

void IdleOp::run(){    
    if (battery.chargerConnected()){        
        dockOp.setInitiatedByOperator(true);
        changeOp(chargeOp);
    }    
}

