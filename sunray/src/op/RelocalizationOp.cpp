// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../StateEstimator.h"
#include "../../events.h"



String RelocalizationOp::name(){
    return "Relocalization";
}


// this operation cannot be exited at once (it has to be completed), we remember operation to call on exit here
void RelocalizationOp::changeOp(Op &anOp, bool returnBackOnExit){
    if (&anOp == this) return;    
    nextOp = &anOp;
}


void RelocalizationOp::begin(){
    nextRelocalizationSecond = 0;
    relocalizationSeconds = 0;
    Logger.event(EVT_LIDAR_RELOCALIZATION);
    motor.stopImmediately(true);       
}


void RelocalizationOp::end(){

}

void RelocalizationOp::run(){
    battery.resetIdle();
    //motor.stopImmediately(true);   
    if (millis() > nextRelocalizationSecond){
        nextRelocalizationSecond = millis() + 1000;  
        relocalizationSeconds++;
        CONSOLE.print("Relocalization (robot must be static)... ");        
        CONSOLE.println(relocalizationSeconds);        
        buzzer.sound(SND_PROGRESS, true);
        if (!gps.isRelocalizing){    
        //if (relocalizationSeconds >= 15){        
        //if (imuCalibrationSeconds >= 9){
            Op::changeOp(*nextOp);
        }
    }           
}


