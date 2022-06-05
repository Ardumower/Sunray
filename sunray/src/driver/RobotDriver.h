// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// motor driver base, battery driver base, bumper driver base

#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include "../../gps.h"
#include <Client.h>

class RobotDriver {
  public:    
    // ---- led states -----           
    bool ledStateWifiInactive;
    bool ledStateWifiConnected;
    bool ledStateGpsFix;
    bool ledStateGpsFloat;
    bool ledStateShutdown;
    bool ledStateError;
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual bool getRobotID(String &id) = 0;
    virtual bool getMcuFirmwareVersion(String &name, String &ver) = 0;    
    virtual float getCpuTemperature() = 0;
};

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
    // read battery temperature (degC) 
    virtual float getBatteryTemperature() = 0;
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

class LiftSensorDriver {
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

class ImuDriver {
  public:
    float roll; // euler radiant
    float pitch; // euler radiant
    float yaw;   // euler radiant
    bool imuFound;   
    // detect module (should update member 'imuFound')
    virtual void detect() = 0;             
    // try starting module with update rate 5 Hz (should return true on success)
    virtual bool begin() = 0;    
    virtual void run() = 0;
    // check if data has been updated (should update members roll, pitch, yaw)
    virtual bool isDataAvail() = 0;
    // reset module data queue (should reset module FIFO etc.)         
    virtual void resetData() = 0;        
};

class BuzzerDriver {
  public:    
    virtual void begin() = 0;
    virtual void run() = 0;
    virtual void noTone() = 0;  	  		      
    virtual void tone(int freq) = 0;
};

class GpsDriver {
  public:
    unsigned long iTOW; //  An interval time of week (ITOW)
    int numSV;         // #signals tracked 
    int numSVdgps;     // #signals tracked with DGPS signal
    double lon;        // deg
    double lat;        // deg
    double height;     // m
    float relPosN;     // m
    float relPosE;     // m
    float relPosD;     // m
    float heading;     // rad
    float groundSpeed; // m/s
    float accuracy;    // m
    float hAccuracy;   // m
    float vAccuracy;   // m
    SolType solution;    
    bool solutionAvail; // should bet set true if received new solution 
    unsigned long dgpsAge;
    unsigned long chksumErrorCounter;
    unsigned long dgpsChecksumErrorCounter;
    unsigned long dgpsPacketCounter;
    // start tcp receiver
    virtual void begin(Client &client, char *host, uint16_t port) = 0;
    // start serial receiver          
    virtual void begin(HardwareSerial& bus,uint32_t baud) = 0;
    // should process receiver data
    virtual void run() = 0;    
    // should configure receiver    
    virtual bool configure() = 0; 
    // should reboot receiver
    virtual void reboot() = 0;
};



#endif

