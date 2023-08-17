// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/*
  BLE 
*/

#ifndef BLE_H // Preprocessor directive to prevent multiple inclusions of this header file
#define BLE_H // Definition to indicate that this header file has been included

#include <Arduino.h> // Include the Arduino core library

class BLEConfig // Definition of the BLEConfig class
{
  public: // Public members of the BLEConfig class
    void run(); // Method to run the BLE configuration
    String read(); // Method to read data from the BLE module
    String exec(String cmd, bool doRetry); // Method to execute a command on the BLE module
  private: // Private members of the BLEConfig class (none defined)
    
};

#endif // End of preprocessor directive to prevent multiple inclusions
