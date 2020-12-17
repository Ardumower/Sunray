// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// external robot (with motor drivers, battery, bumper etc.) connected and controlled via serial line

#ifndef AM_SERIAL_ROBOT_H
#define AM_SERIAL_ROBOT_H

#include <Arduino.h>
#include "MotorDriver.h"
#include "BatteryDriver.h"
#include "BumperDriver.h"


class SerialRobot {
  public:
    unsigned long encoderTicksLeft;
    unsigned long encoderTicksRight;
    unsigned long encoderTicksMow;
    bool receivedEncoders;
    bool motorFault;
    float batteryVoltage;
    float chargeVoltage;
    float chargeCurrent;
    bool triggeredLeftBumper;
    bool triggeredRightBumper;
    bool triggeredLift;
    bool triggeredRain;
    bool triggeredStopButton;           
    void begin();
    void run();
    void requestMotorPwm(int leftPwm, int rightPwm, int mowPwm);
    void requestSummary();
  protected:
    String cmd;
    String cmdResponse;
    unsigned long nextSummaryTime;
    void sendRequest(String req);
    void processComm();
    void processResponse(bool checkCrc);
    void motorResponse();
    void summaryResponse();
};

class SerialMotorDriver: public MotorDriver {
  public:        
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight; 
    SerialRobot &serialRobot;
    bool started;
    SerialMotorDriver(SerialRobot &sr);
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
    SerialRobot &serialRobot;
    SerialBatteryDriver(SerialRobot &sr);
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
    SerialRobot &serialRobot;
    SerialBumperDriver(SerialRobot &sr);
    void begin() override;
    void run() override;
    bool obstacle() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};



#endif
