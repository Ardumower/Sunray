// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "../../config.h"

#ifdef DRV_SIM_ROBOT

#include "test.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"
#include "../../helper.h"
#ifdef __linux__  
  #include <Process.h>
#endif

SessionTest sessionTest;


String Test::name(){
    return "Test";
}

void Test::begin(){

}

void Test::run(){

}

void Test::end(){
}        

void Test::speak(String text){
  #ifdef __linux__
    Process p;
    //String s = "say '" + text + "'";  // sudo apt-get install gnustep-gui-runtime
    //String s = "echo '" + text + "' | festival --tts";  // sudo apt-get install festival
    //String s = "spd-say '" + text + "'";  // sudo apt-get install speech-dispatcher    
    String s = "espeak -vmb-en1 '" + text + "'";  // sudo apt-get install espeak, sudo apt-get install mbrola-en1    
    p.runShellCommand(s+ " &");    
  #endif
}

// --------- SessionTest ---------------------


String SessionTest::name(){
    return "SessionTest";
}

void SessionTest::begin(){
    waypointCounter = 0;
    currTargetX = currTargetY = 0;

    nextBumperTime = millis() + 30000;
    bumperTimeout = 0;

    nextGpsJumpTime = millis() + 75000;
    gpsJumpTimeout = 0;

    nextGpsSignalLossTime = millis() + 95000;
    gpsSignalLossTimeout = 0;

    nextGoDockVoltageTime = millis() + 155000;
}

void SessionTest::run(){

  /*Point target = maps.targetPoint;  
  float dist = distance(currTargetX, currTargetY, target.x(), target.y());
  if (dist > 0){         
    currTargetX = target.x();
    currTargetY = target.y();    
    if (waypointCounter % 2 == 0){
      float robotDist = distance(currTargetX, currTargetY, stateX, stateY);
      if (robotDist > 1.0){
        speak("obstacle");
        robotDriver.setObstacle(currTargetX, currTargetY, 0.2);
      }
    }
    waypointCounter++;
  }*/
  

  /*if (millis() > nextBumperTime){
    nextBumperTime = millis() + 30000; 
    bumperTimeout = millis() + 15000;
    CONSOLE.println("SIM: TRIGGER BUMPER");
    speak("bumper");
    bumper.setSimTriggered(true);
    motorDriver.setSimNoRobotYawRotation(true);
  } else {
    if (millis() > bumperTimeout) {
      bumper.setSimTriggered(false);
      motorDriver.setSimNoRobotYawRotation(false);
    }
  }*/

  /*
  if (millis() > nextGpsJumpTime){
    nextGpsJumpTime = millis() + 75000; 
    gpsJumpTimeout = millis() + 5000;
    CONSOLE.println("SIM: TRIGGER GPS JUMP");
    speak("gps jump");
    gps.setSimGpsJump(true);
  } else {
    if (millis() > gpsJumpTimeout) gps.setSimGpsJump(false);
  }

  if (millis() > nextGpsSignalLossTime){
    nextGpsSignalLossTime = millis() + 95000; 
    gpsSignalLossTimeout = millis() + 15000;
    CONSOLE.println("SIM: TRIGGER GPS SIGNAL LOSS");
    speak("gps loss");
    gps.setSimSolution(SOL_INVALID);
  } else {
    if (millis() > gpsSignalLossTimeout) gps.setSimSolution(SOL_FIXED);
  }

  if (millis() > nextGoDockVoltageTime){
    nextGoDockVoltageTime = millis() + 155000; 
    CONSOLE.println("SIM: TRIGGER DOCK VOLTAGE");
    speak("dock voltage");
    batteryDriver.setSimGoDockVoltage(true);
  } */

}

void SessionTest::end(){
}        

#endif // DRV_SIM_ROBOT

