// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// test cases


#ifndef TEST_H
#define TEST_H

#include "../../config.h"

#ifdef DRV_SIM_ROBOT

#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

  
/* test cases:
1) obstacle avoidance (bumper): if bumper triggered, robot should reach next (or next but one) target point (perform obstacle avoidance)
2) gps no-motion: robot should reach next (or next but one) target point (perform obstacle avoidance)
3) gps signal loss : robot should stop (also mow motor) and continue after gps signal is available again
4) gps fix timeout: robot should stop (also mow motor) and continue after gps fix solution is available again
5) kidnap: robot should reboot gps and continue after undo kidnap
6) imu no-rotation: robot should drive backwards/forwards
7) motor overload timeout: robot should go into error
8) undervoltage: robot should stop
9) dock-voltage: robot should reach charging point
10) docking (bumper): robot should retry (restart) docking 
11) ...

*/


class Test {
  public:         
    bool started;
    bool shouldStop;
    bool succeeded;  
    unsigned long startTime;  
    Test();
    virtual String name();
    virtual void begin();
    virtual void run();
    virtual void end();
    virtual float duration();
    virtual void setSucceeded(bool flag);    
    virtual void speak(String text);      
};

class ObstacleAvoidanceTest: public Test {
  public:
    unsigned long imuFailureTime;
    float targetX;
    float targetY;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

class MotorFaultTest: public Test {
  public:
    unsigned long nextFaultTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

class BumperTest: public Test {
  public:
    int lastGridX;
    int lastGridY;
    bool triggerAllowed;
    unsigned long nextBumperTime;
    unsigned long releaseBumperTime;    
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
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

class Tester {
  public:
    unsigned long nextTestTime;
    virtual void begin();
    virtual void run();
};

extern Tester tester;


#endif // #ifdef DRV_SIM_ROBOT

#endif // #ifndef TEST_H


