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
        // special case: when docking, robot might shortly enter IDLE state before CHARGE state and we should not flag operator mode then        
        // normal case: when going from IDLE to CHARGE state, flag operator mode
        if (millis() - startTime > 2000) {
            CONSOLE.println("IDLE->CHARGE: idle time more than 2secs => assuming robot is not in dock");
            dockOp.setInitiatedByOperator(true);
            battery.setIsDocked(false);            
        }        
        if (initiatedByOperator) dockOp.setInitiatedByOperator(true); // manual stop => manual dock
        changeOp(chargeOp);
    }    
}

