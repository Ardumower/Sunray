// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;


class Motor {
  public:
    float robotPitch;  // robot pitch (rad)
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    float ticksPerCm;  // ticks per cm
    bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload; 
    bool enableMowMotor;
    bool odometryError;    
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    int  pwmMax;
    int  pwmMaxMow;    
    unsigned long motorLeftTicks;
    unsigned long motorRightTicks;
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    float motorMowSense;  // mower motor current (amps)         
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)
    float motorMowSenseLP;  // mower motor current (amps, low-pass)       
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm; 
    float motorRightSenseLPNorm;
    unsigned long motorMowSpinUpTime;
    void begin();
    void run();      
    void test();
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
    void setMowState(bool switchOn);   
    void stopImmediately(bool includeMowerMotor);
  protected: 
    float motorLeftRpmSet; // set speed
    float motorRightRpmSet;    
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorLeftRpmLast;
    float motorRightRpmLast;
    bool motorMowForwardSet; 
    float motorMowPWMSet;  
    float motorMowPWMCurr; 
    int motorLeftPWMCurr;
    int motorRightPWMCurr;    
    float motorMowPWMCurrLP; 
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP;    
    unsigned long lastControlTime;    
    unsigned long nextSenseTime;            
    bool resetMotorFault;
    int resetMotorFaultCounter;
    unsigned long nextResetMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;    
    PID motorLeftPID;
    PID motorRightPID;        
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
    bool checkFault();
    void sense();
    void dumpOdoTicks(int seconds);
    
};


#endif
