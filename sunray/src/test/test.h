// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef TEST_H
#define TEST_H

#include "../../config.h"

#ifdef DRV_SIM_ROBOT

#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

    

// tests

class Test {
  public:         
    virtual String name();
    virtual void begin();
    virtual void run();
    virtual void end();  
    virtual void speak(String text);      
};


class SessionTest: public Test {
  public:        
    int waypointCounter;
    float currTargetX;
    float currTargetY;
    
    unsigned long nextBumperTime;
    unsigned long bumperTimeout;
    
    unsigned long nextGpsJumpTime;
    unsigned long gpsJumpTimeout;

    unsigned long nextGpsSignalLossTime;
    unsigned long gpsSignalLossTimeout;

    unsigned long nextGoDockVoltageTime;    

    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

extern SessionTest sessionTest;


#endif // #ifdef DRV_SIM_ROBOT

#endif // #ifndef TEST_H


