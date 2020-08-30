// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "bumper.h"
#include "config.h"


volatile bool pressed = false;


void BumperLeftInterruptRoutine(){
  pressed = (digitalRead(pinBumperLeft) == LOW);  
}

void BumperRightInterruptRoutine(){
  pressed = (digitalRead(pinBumperRight) == LOW);  
}


void Bumper::begin(){	
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
	if (BUMPER_ENABLE){
    attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	  attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
  }
}

bool Bumper::obstacle(){
  return pressed;
}

void Bumper::run(){  
}

