// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower PCB drivers (Ardumower motor drivers, battery, bumper etc.)

#ifndef AM_ROBOT_DRIVER_H
#define AM_ROBOT_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../../SparkFunHTU21D.h"


#ifndef __linux__


// struct DriverChip defines logic levels how a motor driver works:
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   nPWM               H     Reverse    
// 1) if pwm pin is normal (PWM) or inverted (nPWM) for forward
// 2) if pwm pin is normal (PWM) or inverted (nPWM) for reverse
// 3) if direction pin is LOW (or HIGH) for forward
// 4) if direction pin is LOW (or HIGH) for reverse
// 5) if fault signal is active high (or low)
// 6) if enable signal is active high (or low)
// 7) if there is a minimum PWM speed to ensure (or zero)
// 8) the PWM frequency it can work with

struct DriverChip {
    char const *driverName;       // name of driver (MC33926 etc.)
    bool forwardPwmInvert;  // forward pin uses inverted pwm?
    bool forwardDirLevel;   // forward pin level
    bool reversePwmInvert;  // reverse pin uses inverted pwm?
    bool reverseDirLevel;   // reverse pin level
    bool usePwmRamp;        // use a ramp to get to PWM value?
    bool faultActive;       // level for fault active (LOW/HIGH)
    bool resetFaultByToggleEnable; // reset a fault by toggling enable? 
    bool enableActive;      // level for enable active (LOW/HIGH)
    bool disableAtPwmZeroSpeed; // disable driver at PWM zero speed? (brake function)
    bool keepPwmZeroSpeed;  // keep zero PWM value (disregard minPwmSpeed at zero speed)?    
    int minPwmSpeed;        // minimum PWM speed to ensure     
    int maxPwmSpeed;        // maximum PWM speed to ensure     
    byte pwmFreq;           // PWM frequency (PWM_FREQ_3900 or PWM_FREQ_29300)
    float adcVoltToAmpOfs;  // ADC voltage to amps (offset)     // current (amps)= ((ADCvoltage + ofs)^pow) * scale
    float adcVoltToAmpScale; // ADC voltage to amps (scale)
    float adcVoltToAmpPow;   // ADC voltage to amps (power of number)   
    //bool drivesMowingMotor; // drives mowing motor?    
};


class AmRobotDriver: public RobotDriver {
  public:
    void begin() override;
    void run() override;
    bool getRobotID(String &id) override;    
    bool getMcuFirmwareVersion(String &name, String &ver) override;
    float getCpuTemperature() override;
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
    int lastLeftPwm;
    int lastRightPwm;
    int lastMowPwm;
    int leftSpeedSign; // current motor direction (1 or -1)
    int rightSpeedSign; // current motor direction (1 or -1)
    int mowSpeedSign; // current motor direction (1 or -1)
    DriverChip MC33926;
    DriverChip DRV8308;
    DriverChip A4931;
    DriverChip BLDC8015A;
    DriverChip JYQD;
    DriverChip CUSTOM;
    DriverChip mowDriverChip;
    DriverChip gearsDriverChip;
    void setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip, int speedSign);    
};

class AmBatteryDriver : public BatteryDriver {
  public:    
    HTU21D myHumidity;    
    void begin() override;
    void run() override;
    
    // read battery voltage
    float getBatteryVoltage() override;
    // read charge voltage
    float getChargeVoltage() override;
    // read charge current
    float getChargeCurrent() override;    
    float getBatteryTemperature() override;
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
    bool getLeftBumper() override;
    bool getRightBumper() override;
	
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

class AmLiftSensorDriver: public LiftSensorDriver {
  public:    
    void begin() override;
    void run() override;
    bool triggered() override;
  protected:
    unsigned long nextControlTime;
    bool isLifted;  	  		    
};

class AmBuzzerDriver: public BuzzerDriver {
  public:    
    void begin() override;
    void run() override;
    void noTone() override;
    void tone(int freq) override;
};

#endif

#endif
