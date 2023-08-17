// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower battery management

#ifndef BATTERY_H // Preprocessor directive to prevent multiple inclusions of this header file
#define BATTERY_H // Definition to indicate that this header file has been included

class Battery { // Definition of the Battery class
  public: // Public members of the Battery class
    bool batMonitor; // Boolean to monitor the battery
    float batGoHomeIfBelow; // Voltage level to trigger going home
    float batFullVoltage; // Voltage level considered as full charge
    float batSwitchOffIfBelow; // Voltage level to switch off the battery
    int batSwitchOffIfIdle; // Time in minutes to switch off the battery if idle
    int enableChargingTimeout; // Timeout for enabling charging
    float batFullCurrent; // Current level considered as full charge
    float batteryVoltage; // Current battery voltage in volts
    float chargingVoltage; // Current charging voltage in volts
    float chargingCurrent; // Current charging current in amps
    bool chargingEnabled; // Boolean to indicate if charging is enabled
    bool chargingCompleted; // Boolean to indicate if charging has been completed
    void begin(); // Method to initialize the battery management
    void run(); // Method to run the battery management
    bool chargerConnected(); // Method to check if the charger is connected
    void enableCharging(bool flag); // Method to enable or disable charging
    bool shouldGoHome(); // Method to check if the robot should go home
    bool chargingHasCompleted(); // Method to check if charging has completed
    bool underVoltage(); // Method to check if the battery is under voltage
    void resetIdle(); // Method to reset the idle state
    void switchOff(); // Method to switch off the battery
  protected: // Protected members of the Battery class
    unsigned long nextBatteryTime; // Next time to check the battery
    bool switchOffByOperator; // Boolean to indicate if switched off by operator
    unsigned long timeMinutes; // Time in minutes for charging
    bool chargerConnectedState; // Boolean to indicate the charger connected state
    bool switchOffAllowedUndervoltage; // Boolean to allow switch off under voltage
    bool switchOffAllowedIdle; // Boolean to allow switch off if idle
    unsigned long switchOffTime; // Time to switch off the battery
    unsigned long chargingStartTime; // Time when charging started
    unsigned long nextCheckTime; // Next time to check the charging state
    unsigned long nextEnableTime; // Next time to enable charging
    unsigned long nextPrintTime; // Next time to print the charging state
};

#endif // End of preprocessor directive to prevent multiple inclusions
