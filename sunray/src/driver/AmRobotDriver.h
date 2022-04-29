// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower PCB drivers (Ardumower motor drivers, battery, bumper etc.)

#ifndef AM_ROBOT_DRIVER_H
#define AM_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"


#ifndef __linux__


// struct DriverChip defines logic levels how a motor driver works:
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   nPWM               H     Reverse    
// 1) if pwm pin is normal (PWM) or inverted (nPWM) for forward
// 2) if pwm pin is normal (PWM) or inverted (nPWM) for reverse
// 3) if direction pin is LOW (or HIGH) for forward
// 4) if direction pin is LOW (or HIGH) for reverse
// 5) if fault signal is active high (or low)
// 6) if enable signal is active high (or low)
// 7) if there is a minimum PWM speed to ensure (or zero)

struct DriverChip {
    char *driverName;       // name of driver (MC33926 etc.)
    bool forwardPwmInvert;  // forward pin uses inverted pwm?
    bool forwardDirLevel;   // forward pin level
    bool reversePwmInvert;  // reverse pin uses inverted pwm?
    bool reverseDirLevel;   // reverse pin level
    bool faultActive;       // level for fault active (LOW/HIGH)
    bool enableActive;      // level for enable active (LOW/HIGH)
    int minPwmSpeed;        // minimum PWM speed to ensure     
    //bool drivesMowingMotor; // drives mowing motor?    
};


class AmRobotDriver {
  public:
    void begin();
    void run();
};


class AmMotorDriver: public MotorDriver {
  public:    
    AmMotorDriver();
    void begin() override;
    void run() override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
  protected:
    DriverChip MC33926;
    DriverChip DRV8308;
    DriverChip A4931;
    DriverChip CUSTOM;
    DriverChip mowDriverChip;
    DriverChip gearsDriverChip;
    void setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip);    
};

class AmBatteryDriver : public BatteryDriver {
  public:    
    void begin() override;
    void run() override;
    
    // read battery voltage
    float getBatteryVoltage() override;
    // read charge voltage
    float getChargeVoltage() override;
    // read charge current
    float getChargeCurrent() override;    
    // enable battery charging
    virtual void enableCharging(bool flag) override;
    // keep system on or power-off
    virtual void keepPowerOn(bool flag) override;
  protected:
  	float batteryFactor;
    float currentFactor;
};

class AmBumperDriver: public BumperDriver {
  public:    
    void begin() override;
    void run() override;
    bool obstacle() override;
        
    // get triggered bumper
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};


class AmStopButtonDriver: public StopButtonDriver {
  public:    
    void begin() override;
    void run() override;
    bool triggered() override;
  protected:
    unsigned long nextControlTime;
    bool pressed;  	  		    
};

class AmRainSensorDriver: public RainSensorDriver {
  public:    
    void begin() override;
    void run() override;
    bool triggered() override;
  protected:
    unsigned long nextControlTime;
    bool isRaining;  	  		    
};

class AmLiftSensorDriver: public LiftSensorDriver {
  public:    
    void begin() override;
    void run() override;
    bool triggered() override;
  protected:
    unsigned long nextControlTime;
    bool isLifted;  	  		    
};

#endif

#endif
