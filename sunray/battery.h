// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower battery management

#ifndef BATTERY_H
#define BATTERY_H



class Battery {
  public:
    bool batMonitor;
    float batGoHomeIfBelow;
    float batFullVoltage;
    float batSwitchOffIfBelow;  // switch off battery if below voltage (Volt)
    int batSwitchOffIfIdle;      // switch off battery if idle (minutes)  
    int enableChargingTimeout;
    float batFullCurrent;
	  float batteryVoltage;   // volts
	  float chargingVoltage;  // volts
	  float chargingCurrent;  // amps
    bool chargingEnabled;
    bool chargingCompleted;
	  void begin();            
    void run();	  
	  bool chargerConnected();
    void enableCharging(bool flag);   	      
    bool shouldGoHome();    
    bool chargingHasCompleted();
    bool underVoltage();
    void resetIdle();
    void switchOff();
  protected:       
    unsigned long nextBatteryTime ;
    bool switchOffByOperator;    
    unsigned long timeMinutes;
		bool chargerConnectedState;
    bool switchOffAllowedUndervoltage;
    bool switchOffAllowedIdle;
    unsigned long switchOffTime;
    unsigned long chargingStartTime;
	  unsigned long nextCheckTime;	  
    unsigned long nextEnableTime;	  
		unsigned long nextPrintTime;	  		
};



#endif
