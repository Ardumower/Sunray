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
    float simObstacleX; // obstacle x
    float simObstacleY; // obstacle y
    float simObstacleRadius; // obstacle radius    
    bool robotIsBumpingIntoObstacle;
    unsigned long simTicksLeft; // robot left encoder 
    unsigned long simTicksRight;  // robot right encoder
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;
    bool getMcuFirmwareVersion(String &name, String &ver) override;    
    float getCpuTemperature() override;
    // simulator specific
    void setSimRobotPosState(float x, float y, float delta);
    void setObstacle(float x, float y, float radius);
    bool pointIsInsideObstacle(float x, float y);
  protected:    
};

class SimMotorDriver: public MotorDriver {
  public:        
    bool simOdometryError;
    bool simMotorLeftFault;
    bool simMotorRightFault;
    bool simMotorMowFault;
    bool simMotorLeftOverload;
    bool simMotorRightOverload;
    bool simMotorMowOverload;
    bool simNoMotion;
    bool simNoRobotYawRotation;
    SimRobotDriver &simRobot;
    SimMotorDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    void setMowHeight(int mowHeightMillimeter) override;
    void setMotorPwm(int leftPwm, int rightPwm, int mowPwm, bool releaseBrakesWhenZero) override;
    void getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault) override;
    void resetMotorFaults()  override;
    void getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) override;
    void getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks) override;
    // ----- simulate errors, sensor triggers ----
    void setSimOdometryError(bool flag);
    void setSimMotorFault(bool leftFlag, bool rightFlag, bool mowFlag);
    void setSimMotorOverload(bool leftFlag, bool rightFlag, bool mowFlag);
    void setSimNoMotion(bool flag);
    void setSimNoRobotYawRotation(bool flag);    
  protected:
    unsigned long lastSampleTime;
    unsigned long lastEncoderTicksLeft;
    unsigned long lastEncoderTicksRight;     
};

class SimBatteryDriver : public BatteryDriver {
  public:   
    bool robotIsAtDockingPoint;
    bool simChargerConnected;
    float simVoltage;
    SimRobotDriver &simRobot;
    SimBatteryDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;    
    float getBatteryVoltage() override;
    float getChargeVoltage() override;
    float getChargeCurrent() override;    
    float getBatteryTemperature() override;
    virtual void enableCharging(bool flag) override;
    virtual void keepPowerOn(bool flag) override;
    // ----- simulate errors, sensor triggers ----
    void setSimUndervoltage(bool flag);
    void setSimGoDockVoltage(bool flag);
    void setSimFullyChargedVoltage(bool flag);
    void setSimChargerConnected(bool flag);
};

class SimBumperDriver: public BumperDriver {
  public:    
    bool simTriggered;
    SimRobotDriver &simRobot;
    SimBumperDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    bool nearObstacle() override;    
    bool obstacle() override;
    bool getLeftBumper() override;
    bool getRightBumper() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
    // ----- simulate errors, sensor triggers ----
    void setSimTriggered(bool flag);
};

class SimStopButtonDriver: public StopButtonDriver {
  public:    
    bool simTriggered;
    SimRobotDriver &simRobot;
    SimStopButtonDriver(SimRobotDriver &sr);
    void begin() override;
    void run() override;
    bool triggered() override;  	  		    
    // ----- simulate errors, sensor triggers ----
    void setSimTriggered(bool flag);
};

class SimRainSensorDriver: public RainSensorDriver {
  public:    
    bool simTriggered;
    SimRobotDriver &simRobot;
    SimRainSensorDriver(SimRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;
    // ----- simulate errors, sensor triggers ----
    void setSimTriggered(bool flag);  
};

class SimLiftSensorDriver: public LiftSensorDriver {
  public:    
    bool simTriggered;
    SimRobotDriver &simRobot;
    SimLiftSensorDriver(SimRobotDriver &sr);    
    void begin() override;
    void run() override;
    bool triggered() override;  
    // ----- simulate errors, sensor triggers ----
    void setSimTriggered(bool flag);
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
    bool simNoData;
    bool simDataTimeout;
    bool simTilt;
    SimRobotDriver &simRobot;
    SimImuDriver(SimRobotDriver &sr);    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;       
    // ----- simulate errors, sensor triggers ----
    void setSimNoData(bool flag);
    void setSimDataTimeout(bool flag);
    void setSimTilt(bool flag);
  protected:
    unsigned long nextSampleTime;
};


class SimGpsDriver : public GpsDriver {
  public:        
    bool simGpsJump;
    SimRobotDriver &simRobot;
    SimGpsDriver(SimRobotDriver &sr);
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
    void send(const uint8_t *buffer, size_t size) override;  
    void sendRTCM(const uint8_t *buffer, size_t size) override;  
    // ----- simulate errors, sensor triggers ----
    void setSimSolution(SolType sol);
    void setSimGpsJump(bool flag);
  protected:
    unsigned long resetTime;
    unsigned long nextSolutionTime;
    float floatX;
    float floatY;
};

#endif
