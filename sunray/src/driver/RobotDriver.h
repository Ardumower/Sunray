// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// motor driver base, battery driver base, bumper driver base

#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H


class MotorDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    
    // set pwm (0-255), positive: forward, negative: backwards
    virtual void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) = 0;
    // get motor faults
    virtual void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) = 0;
    // reset motor faults
    virtual void resetMotorFaults() = 0;
    // get motor currents (ampere)
    virtual void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) = 0;
    // get motor encoder ticks
    virtual void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) = 0; 
};



class BatteryDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    
    // read battery voltage
    virtual float getBatteryVoltage() = 0;
    // read charge voltage
    virtual float getChargeVoltage() = 0;
    // read charge current (amps)
    virtual float getChargeCurrent() = 0;
    // enable battery charging
    virtual void enableCharging(bool flag) = 0;
    // keep system on or power-off
    virtual void keepPowerOn(bool flag) = 0;  	  		    
};


class BumperDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool obstacle() = 0;
    
    // get triggered bumper
    virtual void getTriggeredBumper(bool &leftBumper, bool &rightBumper) = 0;  	  		    
};

class StopButtonDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool triggered() = 0;  	  		    
};

class RainSensorDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool triggered() = 0;  	  		    
};



#endif

