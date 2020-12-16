// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// external robot (with motor drivers, battery, bumper etc.) connected and controlled via serial line

#ifndef AM_SERIAL_ROBOT_H
#define AM_SERIAL_ROBOT_H


#include "MotorDriver.h"
#include "BatteryDriver.h"
#include "BumperDriver.h"


class SerialRobotComm {
  public:
    void begin();
    void run();
};

class SerialMotorDriver: public MotorDriver {
  public:    
    SerialMotorDriver();
    void begin() override;
    void run() override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
};

class SerialBatteryDriver : public BatteryDriver {
  public:    
    void begin() override;
    void run() override;    
    float getBatteryVoltage() override;
    float getChargeVoltage() override;
    float getChargeCurrent() override;    
    virtual void enableCharging(bool flag) override;
    virtual void keepPowerOn(bool flag) override;
};

class SerialBumperDriver: public BumperDriver {
  public:    
    void begin() override;
    void run() override;
    bool obstacle() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};



#endif
