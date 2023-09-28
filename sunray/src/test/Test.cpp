// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "../../config.h"

#ifdef DRV_SIM_ROBOT

#include "test.h"
#include "PathFinderTest.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../LineTracker.h"
#include "../../map.h"
#include "../../helper.h"
#ifdef __linux__  
  #include <Process.h>
#endif

ObstacleAvoidanceTest obstacleAvoidanceTest;
MotorFaultTest motorFaultTest;
SessionTest sessionTest;
BumperTest bumperTest;
Tester tester;
//Test &currentTest = obstacleAvoidanceTest; 
Test &currentTest = bumperTest;
PathFinderTest pathFinderTest;


Test::Test(){
  started = false;
  shouldStop = false;
  succeeded = false;
  startTime = 0;
}

String Test::name(){
    return "Test";
}

void Test::begin(){

}

void Test::run(){

}


float Test::duration(){
  return (float(millis() - startTime)) / 1000.0;
}

void Test::setSucceeded(bool flag){
  shouldStop = true;
  succeeded = flag;
}


void Test::end(){
}        

void Test::speak(String text){
  #ifdef __linux__
    Process p;
    //String s = "say '" + text + "'";  // sudo apt-get install gnustep-gui-runtime
    //String s = "echo '" + text + "' | festival --tts";  // sudo apt-get install festival
    //String s = "spd-say '" + text + "'";  // sudo apt-get install speech-dispatcher    
    String s = "espeak -s 150 -p 30 -vmb-en1 '" + text + "'";  // sudo apt-get install espeak, sudo apt-get install mbrola-en1    
    p.runShellCommand(s+ " &");    
  #endif
}
// ---------------------------------------------


String ObstacleAvoidanceTest::name(){
  return "ObstacleAvoidanceTest";
}

void ObstacleAvoidanceTest::begin(){  
  //speak("Obstacle Avoidance Test");
  targetX = targetY = 99999;
  imuFailureTime = millis() + 600000;
}

void ObstacleAvoidanceTest::end(){
}
 
void ObstacleAvoidanceTest::run(){  
  if (robotDriver.simObstacleRadius < 0.01){
    // set obstacle at target if far away from target
    Point target = maps.targetPoint;  
    targetX = target.x();
    targetY = target.y();    
    float robotDist = distance(targetX, targetY, stateX, stateY);
    if (robotDist > 2.0){        
      robotDriver.setObstacle(targetX + (stateX-targetX)/2, targetY + (stateY-targetY)/2, 0.2);
      speak("Obstacle Avoidance Test");
      startTime = millis();
    }
  } else {
    // check if target reached
    float robotDist = distance(targetX, targetY, stateX, stateY);
    if (robotDist < 0.25){        
      speak("succeeded");
      setSucceeded(true);
    }
  }
  if (robotDriver.robotIsBumpingIntoObstacle){
    if (!bumperDriver.simTriggered){
      speak("trigger bumper");
      bumperDriver.setSimTriggered(true);      
      startTime = millis();
      imuFailureTime = millis() + 200;      
    }
  } else {
    bumperDriver.setSimTriggered(false);
  }
  if (duration() > 60.0) {
    speak("failed");
    setSucceeded(false);
  }
  if (millis() > imuFailureTime){
    imuFailureTime = millis() + 600000;
    imuDriver.setSimDataTimeout(true);
  }
}


// --------------------------------------------

String MotorFaultTest::name(){
  return "MotorFaultTest";
}

void MotorFaultTest::begin(){  
  //speak("Motor Fault Test");  
  nextFaultTime = millis() + 60000;
  speak("Motor Fault Test");
}

void MotorFaultTest::end(){
}
 
void MotorFaultTest::run(){    
  if (millis() > nextFaultTime){
    nextFaultTime = millis() + 500;
    motorDriver.setSimMotorFault(false, false, true);  // left, right, mow
  }
}


// --------------------------------------------

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

// --------------------------------------------

String BumperTest::name(){
  return "BumperTest";
}

void BumperTest::begin(){  
  nextBumperTime = millis() + 60000;
  triggerAllowed = true;
  lastGridX = 0;
  lastGridY = 0;
  speak("Bumper Test");
}

void BumperTest::end(){
}
 
void BumperTest::run(){    
  //CONSOLE.println(maps.wayMode);
  //if ((stateOp == OP_MOW) 
  if (maps.wayMode != WAY_DOCK){
    int gridx = robotDriver.simX; // convert into meter-grid
    int gridy = robotDriver.simY; // convert into meter-grid

    if ((gridx != lastGridX) || (gridy != lastGridY)){
      CONSOLE.print("SIM: grid ");
      CONSOLE.print(gridx);
      CONSOLE.print(",");
      CONSOLE.println(gridy);
    

      if ((gridx % 4 == 0) && (gridy % 4 == 0)){  // obstacle grid (every 4x4m)
        releaseBumperTime = millis() + 1000;
        speak("trigger bumper");    
        CONSOLE.println("SIM: trigger bumper");
        bumperDriver.setSimTriggered(true);                            
      }
      lastGridX = gridx;
      lastGridY = gridy;      
    }
    if (millis() > releaseBumperTime){
      bumperDriver.setSimTriggered(false);          
    }
  }
}

// ----------------------------------

void Tester::begin(){
  //currentTest.begin();
  nextTestTime = 0;
}

void Tester::run(){
 
  // if you want to run a test only at certain time intervals...

  /*if (millis() > nextTestTime){
    //maps.generateRandomMap();    
    //pathFinderTest.runPerimeterPathTest();
    
    //pathFinderTest.runRandomPathTest();
    nextTestTime = millis() + 20000;         
  }*/


  // to run a test permanently...
  //currentTest.run();
  
  
  /*if (currentTest.started){
    if (currentTest.shouldStop){ 
       
    } else {
      currentTest.run();
    }
  } else {
    currentTest.startTime = millis();
    currentTest.shouldStop = false;
    currentTest.started = true;
  } */
}


#endif // DRV_SIM_ROBOT

