 // Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "battery.h"
#include "config.h"
#include "helper.h"
#include "motor.h"
#include "robot.h"
#include "buzzer.h"
#include <Arduino.h>

// lithium akkus sollten bis zu einem bestimmten wert CC also constant current geladen werden 
// und danach mit CV constant voltage
// um die lebensdauer zu erhöhen kann man die max spannung herabsetzen
// wir verwenden im Ardumower ein 7S (7 * 4.2V)
// voll geladen ist dann bei 29.4V schluss
// wenn man z.B. die spannung von 4.2V pro zelle auf 4.1V herab setzt (85–90% charged) kann man die lebensdauer 
// verdoppeln - das gleiche gilt bei der endladung

// lithium cells
// ardumower:  Sony US18650 VTC5, 7 cells in series, nominal volage 3.6v
// alfred:   Samsung INR18650-15M, 7 cells in series, nominal voltage 3.6v 

void Battery::begin()
{  
  startupPhase = 0;
  nextBatteryTime = 0;
  nextCheckTime = 0;
  nextEnableTime = 0;
  batteryVoltageSlopeLowCounter = 0;
  nextSlopeTime = 0;
	timeMinutes=0;  
  chargingVoltage = 0;
  chargingCompletedDelay =0;
  batteryVoltage = 0;
  chargerConnectedState = false;
  badChargerContactState = false;      
  chargingCompleted = false;
  chargingEnabled = true;
  docked = false;

  batMonitor = true;              // monitor battery and charge voltage?      
  batGoHomeIfBelow = GO_HOME_VOLTAGE; // 21.5  drive home voltage (Volt)  
  batSwitchOffIfBelow = BAT_UNDERVOLTAGE;  // switch off battery if below voltage (Volt)  
  batSwitchOffIfIdle = 300;      // switch off battery if idle (seconds)
  // The battery will charge if both battery voltage is below that value and charging current is above that value.
  batFullCurrent  = BAT_FULL_CURRENT;  // 0.2  current flowing when battery is fully charged (A)
  batFullVoltage = BAT_FULL_VOLTAGE;  //28.7  voltage when battery is fully charged (we charge to only 90% to increase battery life time)
  enableChargingTimeout = 60 * 30; // if battery is full, wait this time before enabling charging again (seconds)
  batteryVoltage = 0;
  batteryVoltageLast = 0;
  batteryVoltageSlope = 0;
  chargingVoltBatteryVoltDiff = 0;
  switchOffByOperator = false;
  switchOffAllowedUndervoltage = BAT_SWITCH_OFF_UNDERVOLTAGE;
  switchOffAllowedIdle = BAT_SWITCH_OFF_IDLE;

  enableCharging(false);
  resetIdle();    
}


// controls charging relay
void Battery::enableCharging(bool flag){
  if (chargingEnabled == flag) return;
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag;
  batteryDriver.enableCharging(flag);      
	nextPrintTime = 0;  	   	   	
}

bool Battery::chargerConnected(){
  return chargerConnectedState;  
}

bool Battery::isDocked(){
  return docked;
}

void Battery::setIsDocked(bool state){
  CONSOLE.print("battery.setIsDocked ");
  CONSOLE.println(state);
  docked = state;
}

bool Battery::badChargerContact(){
  return badChargerContactState;
}
 
bool Battery::chargingHasCompleted(){
  return chargingCompleted;
}
 

bool Battery::shouldGoHome(){
  if (startupPhase < 2) return false;  
  return (batteryVoltage < batGoHomeIfBelow);
}

bool Battery::underVoltage(){
  if (startupPhase < 2) return false;
  return (batteryVoltage < batSwitchOffIfBelow);
}

void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void Battery::switchOff(){
  CONSOLE.println("switching-off battery by operator...");
  switchOffByOperator = true;
}

void Battery::run(){  
  if (startupPhase == 0) {
    // give some time to establish communication to external hardware etc.
    nextBatteryTime = millis() + 2000;
    startupPhase++;
    return;
  }
  if (millis() < nextBatteryTime) return;    
  nextBatteryTime = millis() + 50;
  if (startupPhase == 1) startupPhase = 2;

  // voltage
  float voltage = batteryDriver.getChargeVoltage();
  if (abs(chargingVoltage-voltage) > 10) {
    chargingVoltage = voltage;
    chargingVoltBatteryVoltDiff = 0;
  }  
  chargingVoltage = 0.9 * chargingVoltage + 0.1* voltage;  

  voltage = batteryDriver.getBatteryVoltage();
  if (abs(batteryVoltage-voltage) > 10) {
    batteryVoltage = voltage;
    batteryVoltageLast = voltage;
    chargingVoltBatteryVoltDiff = 0;
  }  
  float w = 0.995;
  if (chargerConnectedState) w = 0.9;
  batteryVoltage = w * batteryVoltage + (1-w) * voltage;  

  // difference charging voltage and battery voltage
  chargingVoltBatteryVoltDiff = 0.99 * chargingVoltBatteryVoltDiff + 0.01 * (chargingVoltage - batteryVoltage);  

  // current
  chargingCurrent = 0.9 * chargingCurrent + 0.1 * batteryDriver.getChargeCurrent();    
	
  if (!chargerConnectedState){
	  if (chargingVoltage > 7){
      chargerConnectedState = true;		    
		  DEBUG(F("CHARGER CONNECTED chgV="));      	                    
      DEBUG(chargingVoltage);      
      DEBUG(F(" batV="));
      DEBUGLN(batteryVoltage);
      buzzer.sound(SND_OVERCURRENT, true);        
    }
  }
	
  if (millis() >= nextCheckTime){    
    nextCheckTime = millis() + 5000;  	   	   	
    if (chargerConnectedState){        
      if (chargingVoltage <= 5){
        chargerConnectedState = false;
        nextEnableTime = millis() + 5000;  	 // reset charging enable time  	   	 
        DEBUG(F("CHARGER DISCONNECTED chgV="));
        DEBUG(chargingVoltage);        
        DEBUG(F(" batV="));
        DEBUGLN(batteryVoltage);                
      }
    }      		
    timeMinutes = (millis()-chargingStartTime) / 1000 /60;
    if (underVoltage()) {
      DEBUG(F("SWITCHING OFF (undervoltage) batV="));
      DEBUG(batteryVoltage);
      DEBUG("<");
      DEBUGLN(batSwitchOffIfBelow);
      buzzer.sound(SND_OVERCURRENT, true);
      if (switchOffAllowedUndervoltage)  batteryDriver.keepPowerOn(false);     
    } else if ((millis() >= switchOffTime) || (switchOffByOperator)) {
      DEBUGLN(F("SWITCHING OFF (idle timeout)"));              
      buzzer.sound(SND_OVERCURRENT, true);
      if ((switchOffAllowedIdle) || (switchOffByOperator)) batteryDriver.keepPowerOn(false);
    } else batteryDriver.keepPowerOn(true);              

    // battery volage slope 
    float w = 0.999; 
    batteryVoltageSlope = w * batteryVoltageSlope + (1-w) * (batteryVoltage - batteryVoltageLast) * 60.0/5.0;   // 5s => 1min  
    batteryVoltageLast = batteryVoltage;
    
    if (millis() >= nextSlopeTime){
      nextSlopeTime = millis() + 60000; // 1 minute
      badChargerContactState = false;
      if (chargerConnectedState){
        if (!chargingCompleted){
          if (chargingVoltBatteryVoltDiff < -3.0){  // allow larger resistance (as mower chassis is getting older) 
          //if (batteryVoltageSlope < 0){
            badChargerContactState = true;
            DEBUG(F("CHARGER BAD CONTACT chgV="));
            DEBUG(chargingVoltage);
            DEBUG(" batV=");
            DEBUG(batteryVoltage);
            DEBUG(" diffV=");
            DEBUG(chargingVoltBatteryVoltDiff);
            DEBUG(" slope(v/min)=");
            DEBUGLN(batteryVoltageSlope);
          }      
        } 
      }
      if (abs(batteryVoltageSlope) < BAT_FULL_SLOPE){
        batteryVoltageSlopeLowCounter = min(10, batteryVoltageSlopeLowCounter + 1);
      } else {
        batteryVoltageSlopeLowCounter = 0; //max(0, batteryVoltageSlopeLowCounter - 1);
      }
    }

		if (millis() >= nextPrintTime){
			nextPrintTime = millis() + 60000;  // 1 minute  	   	   	
			//if (chargerConnectedState){      
        //print();			
        DEBUG(F("charger conn="));
        DEBUG(chargerConnected());
        DEBUG(F(" chgEnabled="));
        DEBUG(chargingEnabled);
        DEBUG(F(" chgTime="));      
        DEBUG(timeMinutes);
        DEBUG(F(" charger: "));      
        DEBUG(chargingVoltage);
        DEBUG(F(" V  "));    
        DEBUG(chargingCurrent);   
        DEBUG(F(" A "));         
        DEBUG(F(" bat: "));
        DEBUG(batteryVoltage);
        DEBUG(F(" V  "));    
        DEBUG(" diffV=");
        DEBUG(chargingVoltBatteryVoltDiff);
        DEBUG(" slope(v/min)=");
        DEBUG(batteryVoltageSlope);
        DEBUG(" slopeLowCounter=");
        DEBUG(batteryVoltageSlopeLowCounter);
        DEBUGLN();      
     // }
    }	
  }
  
  if (millis() > nextEnableTime){
    nextEnableTime = millis() + 5000;  	   	   	
    
    if (chargerConnectedState){	      
        // charger in connected state
        if (chargingEnabled){
          //if ((timeMinutes > 180) || (chargingCurrent < batFullCurrent)) {   
          // https://github.com/Ardumower/Sunray/issues/32               
          if (chargingCompletedDelay > 5) {  // chargingCompleted check first after 6 * 5000ms = 30sec. 
            chargingCompleted = ((chargingCurrent <= batFullCurrent) || (batteryVoltage >= batFullVoltage) || (batteryVoltageSlopeLowCounter > 5)); 
          } 
          else {           
            chargingCompletedDelay++;  
          }          
          if (chargingCompleted) {
            // stop charging
            nextEnableTime = millis() + 1000 * enableChargingTimeout;   // check charging current again in 30 minutes
            chargingCompleted = true;
            enableCharging(false);
          } 
        } else {
           //if (batteryVoltage < startChargingIfBelow) {
              // start charging
              enableCharging(true);
              chargingStartTime = millis();  
          //}        
        }    
    }
    else { 
      // reset to avoid direct undocking after docking   
      chargingCompleted       = false; 
      chargingCompletedDelay  = 0;  // reset chargingCompleteted delay counter 
    } 

  }


}
