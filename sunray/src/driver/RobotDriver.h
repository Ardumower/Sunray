// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// motor driver base, battery driver base, bumper driver base

#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H


class MotorDriver {
  public:    
    virtual void begin();
    virtual void run();
    
    // set pwm (0-255), positive: forward, negative: backwards
    virtual void setMotorPwm(int leftPwm, int rightPwm, int mowPwm);
    // get motor faults
    virtual void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault);
    // reset motor faults
    virtual void resetMotorFaults();
    // get motor currents (ampere)
    virtual void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent);
    // get motor encoder ticks
    virtual void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks); 
};



class BatteryDriver {
  public:    
    virtual void begin();
    virtual void run();
    
    // read battery voltage
    virtual float getBatteryVoltage();
    // read charge voltage
    virtual float getChargeVoltage();
    // read charge current (amps)
    virtual float getChargeCurrent();
    // enable battery charging
    virtual void enableCharging(bool flag);
    // keep system on or power-off
    virtual void keepPowerOn(bool flag);  	  		    
};


class BumperDriver {
  public:    
    virtual void begin();
    virtual void run();
    virtual bool obstacle();
    
    // get triggered bumper
    virtual void getTriggeredBumper(bool &leftBumper, bool &rightBumper);  	  		    
};

class StopButtonDriver {
  public:    
    virtual void begin();
    virtual void run();
    virtual bool triggered();  	  		    
};


#endif

