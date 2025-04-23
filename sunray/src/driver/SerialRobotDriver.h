// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Alfred mower: external robot (with motor drivers, battery, bumper etc.) connected and controlled via serial line

#ifndef SERIAL_ROBOT_DRIVER_H
#define SERIAL_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#ifdef __linux__
  #include <Process.h>
#endif


class SerialRobotDriver: public RobotDriver {
  public:
    String robotID;
    String mcuFirmwareName;
    String mcuFirmwareVersion;
    int requestLeftPwm;
    int requestRightPwm;
    int requestMowPwm;        
    unsigned long encoderTicksLeft;
    unsigned long encoderTicksRight;
    unsigned long encoderTicksMow;
    bool mcuCommunicationLost;
    bool motorFault;
    float batteryVoltage;
    float chargeVoltage;
    float chargeCurrent;
    float mowCurr;
    float motorLeftCurr;
    float motorRightCurr;
    bool resetMotorTicks;
    float batteryTemp;
    float cpuTemp;
    bool triggeredLeftBumper;
    bool triggeredRightBumper;
    bool triggeredLift;
    bool triggeredRain;
    bool triggeredStopButton;
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;
    bool getMcuFirmwareVersion(String &name, String &ver) override;
    float getCpuTemperature() override;
    void requestMotorPwm(int leftPwm, int rightPwm, int mowPwm);
    void requestSummary();
    void requestVersion();
    void updatePanelLEDs();
    void updateCpuTemperature();
    void updateWifiConnectionState();
    bool setLedState(int ledNumber, bool greenState, bool redState);
    bool setFanPowerState(bool state);
    bool setImuPowerState(bool state);
  protected:    
    bool ledPanelInstalled;
    #ifdef __linux__
      Process cpuTempProcess;
      Process wifiStatusProcess;    
    #endif
    String cmd;
    String cmdResponse;
    unsigned long nextMotorTime;    
    unsigned long nextSummaryTime;
    unsigned long nextConsoleTime;
    unsigned long nextTempTime;
    unsigned long nextWifiTime;
    unsigned long nextLedTime;
    int cmdMotorCounter;
    int cmdSummaryCounter;
    int cmdMotorResponseCounter;
    int cmdSummaryResponseCounter;
    void sendRequest(String s);
    void processComm();
    void processResponse(bool checkCrc);
    void motorResponse();
    void summaryResponse();
    void versionResponse();
};

class SerialMotorDriver: public MotorDriver {
  public:        
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight; 
    unsigned long lastEncoderTicksMow;     
    SerialRobotDriver &serialRobot;
    SerialMotorDriver(SerialRobotDriver &sr);
    void begin() override;
    void run() override;
    void setMowHeight(int mowHeightMillimeter) override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm, bool releaseBrakesWhenZero) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
};

class SerialBatteryDriver : public BatteryDriver {
  public:   
    float batteryTemp;
    bool mcuBoardPoweredOn;
    unsigned long nextTempTime;
    unsigned long nextADCTime;
    bool adcTriggered;
    unsigned long linuxShutdownTime;
    #ifdef __linux__
      Process batteryTempProcess;
    #endif
    SerialRobotDriver &serialRobot;
    SerialBatteryDriver(SerialRobotDriver &sr);
    void begin() override;
    void run() override;    
    float getBatteryVoltage() override;
    float getChargeVoltage() override;
    float getChargeCurrent() override;    
    float getBatteryTemperature() override;    
    virtual void enableCharging(bool flag) override;
    virtual void keepPowerOn(bool flag) override;
    void updateBatteryTemperature();
};

class SerialBumperDriver: public BumperDriver {
  public:    
    SerialRobotDriver &serialRobot;
    SerialBumperDriver(SerialRobotDriver &sr);
    void begin() override;
    void run() override;
    bool nearObstacle() override;
    bool obstacle() override;
    bool getLeftBumper() override;
    bool getRightBumper() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};

class SerialStopButtonDriver: public StopButtonDriver {
  public:    
    SerialRobotDriver &serialRobot;
    SerialStopButtonDriver(SerialRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;  	  		    
};

class SerialRainSensorDriver: public RainSensorDriver {
  public:    
    SerialRobotDriver &serialRobot;
    SerialRainSensorDriver(SerialRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class SerialLiftSensorDriver: public LiftSensorDriver {
  public:    
    SerialRobotDriver &serialRobot;
    SerialLiftSensorDriver(SerialRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class SerialBuzzerDriver: public BuzzerDriver {
  public:    
    SerialRobotDriver &serialRobot;
    SerialBuzzerDriver(SerialRobotDriver &sr);    
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;  
};


#endif
