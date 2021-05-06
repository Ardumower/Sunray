// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower PCB drivers (Ardumower motor drivers, battery, bumper etc.)

#ifndef AM_ROBOT_DRIVER_H
#define AM_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"


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
    void setMC33926(int pinDir, int pinPWM, int speed);
    void setBrushless(int pinDir, int pinPWM, int speed);
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


#endif
