// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// owlRobotics platform mower: robot (with motor drivers, battery, bumper etc.) connected and controlled via CAN bus

#ifndef CAN_ROBOT_DRIVER_H
#define CAN_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#ifdef __linux__
  #include <Process.h>
  #include "../../linuxcan.h"
#endif


class CanRobotDriver: public RobotDriver {
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
      LinuxCAN can;
      Process cpuTempProcess;
      Process wifiStatusProcess;          
    #else  
      CAN can; // dummy, so compiler doesn't complain on other platforms
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

class CanMotorDriver: public MotorDriver {
  public:        
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight; 
    unsigned long lastEncoderTicksMow;     
    CanRobotDriver &canRobot;
    CanMotorDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
};

class CanBatteryDriver : public BatteryDriver {
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
    CanRobotDriver &canRobot;
    CanBatteryDriver(CanRobotDriver &sr);
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

class CanBumperDriver: public BumperDriver {
  public:    
    CanRobotDriver &canRobot;
    CanBumperDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    bool obstacle() override;
    bool getLeftBumper() override;
    bool getRightBumper() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};

class CanStopButtonDriver: public StopButtonDriver {
  public:    
    CanRobotDriver &canRobot;
    CanStopButtonDriver(CanRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;  	  		    
};

class CanRainSensorDriver: public RainSensorDriver {
  public:    
    CanRobotDriver &canRobot;
    CanRainSensorDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class CanLiftSensorDriver: public LiftSensorDriver {
  public:    
    CanRobotDriver &canRobot;
    CanLiftSensorDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class CanBuzzerDriver: public BuzzerDriver {
  public:    
    CanRobotDriver &canRobot;
    CanBuzzerDriver(CanRobotDriver &sr);    
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;  
};


#endif
