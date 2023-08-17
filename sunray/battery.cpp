// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "battery.h" // Include battery header file
#include "config.h"  // Include configuration header file
#include "helper.h"  // Include helper functions header file
#include "motor.h"   // Include motor control header file
#include "robot.h"   // Include robot control header file
#include "buzzer.h"  // Include buzzer control header file
#include <Arduino.h> // Include Arduino standard library

// Comment in German about lithium batteries and charging strategy

void Battery::begin()
{
  nextBatteryTime = 0; // Initialize next battery check time
  nextCheckTime = 0;   // Initialize next check time
  nextEnableTime = 0;  // Initialize next enable time
  timeMinutes = 0;     // Initialize time in minutes
  chargingVoltage = 0; // Initialize charging voltage
  batteryVoltage = 0;  // Initialize battery voltage
  chargerConnectedState = false; // Initialize charger connected state
  chargingCompleted = false;     // Initialize charging completed state
  chargingEnabled = true;        // Initialize charging enabled state
  batMonitor = true;             // Enable battery monitoring
  batGoHomeIfBelow = GO_HOME_VOLTAGE; // Set go home voltage
  batSwitchOffIfBelow = 18.9;    // Set switch off voltage
  batSwitchOffIfIdle = 300;      // Set switch off idle time
  batFullCurrent  = BAT_FULL_CURRENT; // Set full current
  batFullVoltage = BAT_FULL_VOLTAGE; // Set full voltage
  enableChargingTimeout = 60 * 30; // Set charging timeout
  batteryVoltage = 0;             // Initialize battery voltage
  switchOffByOperator = false;    // Initialize switch off by operator flag
  switchOffAllowedUndervoltage = BAT_SWITCH_OFF_UNDERVOLTAGE; // Set switch off allowed under voltage
  switchOffAllowedIdle = BAT_SWITCH_OFF_IDLE; // Set switch off allowed idle
  enableCharging(false);          // Disable charging
  resetIdle();                    // Reset idle state
}

// Function to enable or disable charging
void Battery::enableCharging(bool flag){
  if (chargingEnabled == flag) return; // Return if charging state is already set
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag; // Set charging enabled state
  batteryDriver.enableCharging(flag); // Enable or disable charging in the battery driver
  nextPrintTime = 0; // Reset print time
}

// Function to check if charger is connected
bool Battery::chargerConnected(){
  return chargerConnectedState; // Return charger connected state
}

// Function to check if charging has completed
bool Battery::chargingHasCompleted(){
  return chargingCompleted; // Return charging completed state
}

// Function to check if robot should go home
bool Battery::shouldGoHome(){
  return (batteryVoltage < batGoHomeIfBelow); // Return true if battery voltage is below go home voltage
}

// Function to check if battery is under voltage
bool Battery::underVoltage(){
  return (batteryVoltage < batSwitchOffIfBelow); // Return true if battery voltage is below switch off voltage
}

// Function to reset idle state
void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000; // Reset switch off time based on idle timeout
}

// Function to switch off battery by operator
void Battery::switchOff(){
  CONSOLE.println("switching-off battery by operator..."); // Print message to console
  switchOffByOperator = true; // Set switch off by operator flag
}

// Main run function for battery management
void Battery::run(){  
  if (millis() < nextBatteryTime) return; // Return if it's not time to check the battery
  nextBatteryTime = millis() + 50; // Set next battery check time
  
  float voltage = batteryDriver.getChargeVoltage(); // Get charge voltage from battery driver
  if (abs(chargingVoltage - voltage) > 10) chargingVoltage = voltage; // Update charging voltage if difference is significant
  chargingVoltage = 0.9 * chargingVoltage + 0.1 * voltage; // Apply smoothing filter to charging voltage

  voltage = batteryDriver.getBatteryVoltage(); // Get battery voltage from battery driver
  if (abs(batteryVoltage - voltage) > 10) batteryVoltage = voltage; // Update battery voltage if difference is significant
  float w = 0.995;
  if (chargerConnectedState) w = 0.9;
  batteryVoltage = w * batteryVoltage + (1 - w) * voltage; // Apply smoothing filter to battery voltage

  chargingCurrent = 0.9 * chargingCurrent + 0.1 * batteryDriver.getChargeCurrent(); // Apply smoothing filter to charging current

  if (!chargerConnectedState){
    if (chargingVoltage > 5){
      chargerConnectedState = true; // Set charger connected state
      DEBUGLN(F("CHARGER CONNECTED")); // Print debug message
      buzzer.sound(SND_OVERCURRENT, true); // Sound buzzer for overcurrent
    }
  }

  if (millis() >= nextCheckTime){    
    nextCheckTime = millis() + 5000; // Set next check time
    if (chargerConnectedState){        
      if (chargingVoltage <= 5){
        chargerConnectedState = false; // Set charger disconnected state
        nextEnableTime = millis() + 5000; // Reset charging enable time
        DEBUGLN(F("CHARGER DISCONNECTED")); // Print debug message
      }
    }      		
    timeMinutes = (millis() - chargingStartTime) / 1000 / 60; // Calculate charging time in minutes
    if (underVoltage()) {
      DEBUGLN(F("SWITCHING OFF (undervoltage)")); // Print debug message for undervoltage
      buzzer.sound(SND_OVERCURRENT, true); // Sound buzzer for overcurrent
      if (switchOffAllowedUndervoltage) batteryDriver.keepPowerOn(false); // Switch off if allowed under voltage
    } else if ((millis() >= switchOffTime) || (switchOffByOperator)) {
      DEBUGLN(F("SWITCHING OFF (idle timeout)")); // Print debug message for idle timeout
      buzzer.sound(SND_OVERCURRENT, true); // Sound buzzer for overcurrent
      if ((switchOffAllowedIdle) || (switchOffByOperator)) batteryDriver.keepPowerOn(false); // Switch off if allowed idle or by operator
    } else batteryDriver.keepPowerOn(true); // Keep power on

    if (millis() >= nextPrintTime){
      nextPrintTime = millis() + 60000; // Set next print time
      // Debug print section (commented out)
    }	
  }
  
  if (millis() > nextEnableTime){
    nextEnableTime = millis() + 5000; // Set next enable time
    if (chargerConnectedState){	      
      // Charger in connected state
      if (chargingEnabled){
        // Charging is enabled
        chargingCompleted = ((chargingCurrent <= batFullCurrent) || (batteryVoltage >= batFullVoltage)); // Check if charging is completed
        if (chargingCompleted) {
          // Stop charging
          nextEnableTime = millis() + 1000 * enableChargingTimeout; // Check charging current again in 30 minutes
          chargingCompleted = true; // Set charging completed flag
          enableCharging(false); // Disable charging
        } 
      } else {
        // Start charging
        enableCharging(true); // Enable charging
        chargingStartTime = millis(); // Set charging start time
      }    
    } 		
  }
}
 