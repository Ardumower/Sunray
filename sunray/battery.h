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
	  float batteryFactor;
    float currentFactor;
    float batGoHomeIfBelow;
    float startChargingIfBelow;
    float batSwitchOffIfBelow;  // switch off battery if below voltage (Volt)
    int batSwitchOffIfIdle;      // switch off battery if idle (minutes)  
    int enableChargingTimeout;
    float batFullCurrent;
	  float batteryVoltage;   // volts
	  float chargingVoltage;  // volts
	  float chargingCurrent;  // amps
    bool chargingEnabled;
	  float ADCRef;
    void begin();            
    void run();	  
	  bool chargerConnected();
    void enableCharging(bool flag);   	  
    void allowSwitchOff(bool flag);      
    bool shouldGoHome();    
    bool underVoltage();
    void resetIdle();
  protected:           
    unsigned long timeMinutes;
		bool chargerConnectedState;
    bool switchOffAllowed;
    unsigned long switchOffTime;
    unsigned long chargingStartTime;
	  unsigned long nextCheckTime;	  
    unsigned long nextEnableTime;	  
		unsigned long nextPrintTime;	  		
};



#endif
