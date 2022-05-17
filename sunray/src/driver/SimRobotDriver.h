// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// robot mower simulation (with simulated motor drivers, battery, bumper etc.) 

#ifndef SIM_ROBOT_DRIVER_H
#define SIM_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"


class SimRobotDriver: public RobotDriver {
  public:
    float simX;  // robot x
    float simY;  // robot y
    float simDelta; // robot yaw        
    float leftSpeed; // left tire speed
    float rightSpeed;  // right tire speed
    float mowSpeed;    // mow motor speed
    float linearSpeed;  // linear speed 
    float angularSpeed;  // angular speed    
    unsigned long simTicksLeft; // robot left encoder 
    unsigned long simTicksRight;  // robot right encoder
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;
    bool getMcuFirmwareVersion(String &name, String &ver) override;    
  protected:    
};

class SimMotorDriver: public MotorDriver {
  public:        
    SimRobotDriver &simRobot;
    SimMotorDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
  protected:
    unsigned long lastSampleTime;
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight;     
};

class SimBatteryDriver : public BatteryDriver {
  public:   
    SimRobotDriver &simRobot;
    SimBatteryDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;    
    float getBatteryVoltage() override;
    float getChargeVoltage() override;
    float getChargeCurrent() override;    
    virtual void enableCharging(bool flag) override;
    virtual void keepPowerOn(bool flag) override;
};

class SimBumperDriver: public BumperDriver {
  public:    
    SimRobotDriver &simRobot;
    SimBumperDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    bool obstacle() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};

class SimStopButtonDriver: public StopButtonDriver {
  public:    
    SimRobotDriver &simRobot;
    SimStopButtonDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;  	  		    
};

class SimRainSensorDriver: public RainSensorDriver {
  public:    
    SimRobotDriver &simRobot;
    SimRainSensorDriver(SimRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class SimLiftSensorDriver: public LiftSensorDriver {
  public:    
    SimRobotDriver &simRobot;
    SimLiftSensorDriver(SimRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
};

class SimBuzzerDriver: public BuzzerDriver {
  public:    
    SimRobotDriver &simRobot;
    SimBuzzerDriver(SimRobotDriver &sr);    
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;  
};

class SimImuDriver: public ImuDriver {    
  public:    
    SimRobotDriver &simRobot;
    SimImuDriver(SimRobotDriver &sr);    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;        
  protected:
    unsigned long nextSampleTime;
};


class SimGpsDriver : public GpsDriver {
  public:        
    SimRobotDriver &simRobot;
    SimGpsDriver(SimRobotDriver &sr);
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
  protected:
    unsigned long nextSolutionTime;
};

#endif
