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


void Battery::begin()
{
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  
  nextCheckTime = 0;
  nextEnableTime = 0;
	timeMinutes=0;  
  chargerConnectedState = false;      
  chargingEnabled = true;
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage
  
  //INA169:  A precision amplifier measures the voltage across the Rs=0.1 ohm, 1% sense resistor. 
  //The Rs is rated for 2W continuous so you can measure up to +5A continuous. 
  //The output is a current that is drawn through the on-board RL=10K+10K=20K resistors so that the 
  // output voltage is 2V per Amp. So for 1A draw, the output will be 2V. You can change the 
  // load resistor RL to be smaller by soldering the bridge If you solder the bridge (RL=10K resistor) 
  // you'll get 1V per Amp.   
  //
  // Is = Vout * 1k / (Rs * RL)
  //   a) bridged      RL=10K:  Is = 1V * 1k / (0.1*10K)  = 1A
  //   b) non-bridged  RL=20k:  Is = 1v * 1k / (0.1*20K)  = 0.5A
  
  currentFactor = CURRENT_FACTOR;         // ADC voltage to current ampere  (0.5 for non-bridged)
  batMonitor = true;              // monitor battery and charge voltage?    
  batGoHomeIfBelow = 21.0;     // drive home voltage (Volt)  
  batSwitchOffIfBelow = 18.9;  // switch off battery if below voltage (Volt)  
  batSwitchOffIfIdle = 300;      // switch off battery if idle (seconds)
  batFullCurrent  = 0.2;      // current flowing when battery is fully charged
  batFullVoltage = 28.7;      // voltage when battery is fully charged (we charge to only 90% to increase battery life time)
  enableChargingTimeout = 60 * 30; // if battery is full, wait this time before enabling charging again (seconds)
  batteryVoltage = 0;

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  
  enableCharging(false);
  allowSwitchOff(false);
  resetIdle();    
}


// controls charging relay
void Battery::enableCharging(bool flag){
  if (chargingEnabled == flag) return;
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag;
  digitalWrite(pinChargeRelay, flag);      
	nextPrintTime = 0;  	   	   	
}

bool Battery::chargerConnected(){
  return chargerConnectedState;  
}
  

bool Battery::shouldGoHome(){
  return (batteryVoltage < batGoHomeIfBelow);
}

bool Battery::underVoltage(){
  return (batteryVoltage < batSwitchOffIfBelow);
}

void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void Battery::allowSwitchOff(bool flag){
  switchOffAllowed = flag;
  if (flag) resetIdle();
}


void Battery::run(){  
  chargingVoltage = ((float)ADC2voltage(analogRead(pinChargeVoltage))) * batteryFactor;  
  float w = 0.99;
  if (batteryVoltage < 5) w = 0;
  batteryVoltage = w * batteryVoltage + (1-w) * ((float)ADC2voltage(analogRead(pinBatteryVoltage))) * batteryFactor;  
  chargingCurrent = 0.9 * chargingCurrent + 0.1 * ((float)ADC2voltage(analogRead(pinChargeCurrent))) * currentFactor;    
		
  if (!chargerConnectedState){
	  if (chargingVoltage > 5){
      chargerConnectedState = true;		    
		  DEBUGLN(F("CHARGER CONNECTED"));      	              
      buzzer.sound(SND_OVERCURRENT, true);        
    }
  }
	
  if (millis() >= nextCheckTime){    
    nextCheckTime = millis() + 5000;  	   	   	
    if (chargerConnectedState){        
      if (chargingVoltage <= 5){
        chargerConnectedState = false;
        nextEnableTime = millis() + 5000;  	 // reset charging enable time  	   	 
        DEBUGLN(F("CHARGER DISCONNECTED"));              				        
      }
    }      		
    timeMinutes = (millis()-chargingStartTime) / 1000 /60;
    if (switchOffAllowed) {
      if (underVoltage()) {
        DEBUGLN(F("SWITCHING OFF (undervoltage)"));              
        buzzer.sound(SND_OVERCURRENT, true);
        digitalWrite(pinBatterySwitch, LOW);    
      } else if (millis() >= switchOffTime) {
        DEBUGLN(F("SWITCHING OFF (idle timeout)"));              
        buzzer.sound(SND_OVERCURRENT, true);
        digitalWrite(pinBatterySwitch, LOW);    
      } else digitalWrite(pinBatterySwitch, HIGH);          
    }
    	      
		if (millis() >= nextPrintTime){
			nextPrintTime = millis() + 60000;  	   	   	
			//print();			
			/*DEBUG(F("charger conn="));
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
			DEBUG(F("switchOffAllowed="));   
			DEBUG(switchOffAllowed);      
			DEBUGLN();      */					
    }	
  }
  
  if (millis() > nextEnableTime){
    nextEnableTime = millis() + 5000;  	   	   	
    if (chargerConnectedState){	      
        // charger in connected state
        if (chargingEnabled){
          //if ((timeMinutes > 180) || (chargingCurrent < batFullCurrent)) {        
          if ((chargingCurrent <= batFullCurrent) || (batteryVoltage >= batFullVoltage)) {
            // stop charging
            nextEnableTime = millis() + 1000 * enableChargingTimeout;   // check charging current again in 30 minutes
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
  }
}
