// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "bumper.h"
#include "config.h"
#include "robot.h"
#include <Arduino.h>

volatile bool inputLeftPressed = false;
volatile bool inputRightPressed = false;

volatile bool outputLeftPressed = false;
volatile bool outputRightPressed = false;

unsigned long leftPressedOnDelay = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long rightPressedOnDelay = 0;

unsigned long bumperStayActivTime = 0;    // duration, the bumper stays triggered
unsigned long lastCallBumperObstacle = 0; // last call for bumper.obstacle


void Bumper::begin(){
  bumperDriver.begin();

  leftPressedOnDelay  = millis();
  rightPressedOnDelay = millis();
}

void Bumper::run() {
  bumperDriver.run();
    
  inputLeftPressed   = bumperDriver.getLeftBumper();
  inputRightPressed  = bumperDriver.getRightBumper();

  if (BUMPER_INVERT){
    inputLeftPressed = !inputLeftPressed;
    inputRightPressed = !inputRightPressed;
  }

  if (BUMPER_ENABLE){
    outputLeftPressed = inputLeftPressed;
    outputRightPressed = inputRightPressed;  
  
    /*
    FIXME: code does not seem to work properly in all cases
    https://github.com/Ardumower/Sunray/pull/103#issuecomment-1215526140
    IDEA: do not check bumper errors here, check them (like for the lift-sensor) in 'src/EscapeReverseOp.cpp/run'

    bool bumperLeft   = false;
    bool bumperRight  = false;

    // delay for the bumper inputs
    if (inputLeftPressed){
      if (millis() >= (leftPressedOnDelay + BUMPER_TRIGGER_DELAY)) bumperLeft = true;
    } else leftPressedOnDelay = millis();

    if (inputRightPressed){
      if (millis() >= (rightPressedOnDelay + BUMPER_TRIGGER_DELAY)) bumperRight = true;
    } else rightPressedOnDelay = millis();

    if (millis() > (linearMotionStartTime + BUMPER_DEADTIME)){
      outputLeftPressed   = bumperLeft;
      outputRightPressed  = bumperRight;
    } else outputLeftPressed = outputRightPressed = false;

    
    // check if bumper stays triggered for a long time periode (maybe blocked)
    if ((bumperRight || bumperLeft) && (BUMPER_MAX_TRIGGER_TIME > 0)){
      if ((abs(motor.linearSpeedSet) >= 0.05) || (abs(motor.angularSpeedSet) >= 0.05)) { // if no movement, bumperStayActivTime paused
        bumperStayActivTime = bumperStayActivTime + (millis()-lastCallBumperObstacle);
      }
      if ((bumperStayActivTime) > (BUMPER_MAX_TRIGGER_TIME * 1000)){ // maximum trigger time reached -> set error
        if (stateOp != OP_ERROR){
          stateSensor = SENS_BUMPER;
          CONSOLE.println("ERROR BUMPER BLOCKED - BUMPER_MAX_TRIGGER_TIME exceeded. See config.h for further information");
          setOperation(OP_ERROR);
        }
      }
    } else bumperStayActivTime = 0;
    */
    lastCallBumperObstacle = millis();
  }
}


bool Bumper::obstacle(){
  if (BUMPER_ENABLE){
    return (outputLeftPressed || outputRightPressed);
  }
  else return false;
}

bool Bumper::nearObstacle(){
  return bumperDriver.nearObstacle();
}

// send separated signals without delay to sensortest
bool Bumper::testLeft(){
  return (inputLeftPressed);
}

bool Bumper::testRight(){
  return (inputRightPressed);
}
