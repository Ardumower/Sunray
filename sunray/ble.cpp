// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Bluetooth BLE/4.0 module (HC-08/HM-10 ZS-040)
// Information and documentation links

#include "ble.h" // Include the header file for BLE functionality
#include <Arduino.h> // Include the Arduino core library
#include "config.h" // Include the configuration file

String BLEConfig::read(){ // Method to read data from the BLE module
  String res; // Result string to store the read data
  unsigned long timeout = millis() + 2000; // Set a timeout for reading
  while (millis() < timeout){ // Loop until timeout
    while (BLE.available()){ // While data is available from BLE
      timeout = millis() + 200; // Reset the timeout
      char ch = BLE.read(); // Read a character from BLE
      CONSOLE.print(ch); // Print the character to the console
      res += ch; // Append the character to the result string
    }
  }
  return res; // Return the result string
}

String BLEConfig::exec(String cmd, bool doRetry){ // Method to execute a command on the BLE module
  String res; // Result string to store the response
  delay(500); // Delay for stability
  for (int retry=0; retry < 3; retry++){ // Retry loop
    CONSOLE.print("BLE: "); // Print debug information
    CONSOLE.print(cmd); // Print the command
    BLE.print(cmd); // Send the command to the BLE module
    res = read(); // Read the response
    if ((res != "") || (!doRetry)) break; // Break if response received or no retry needed
    CONSOLE.print("retry "); // Print retry information
    CONSOLE.print(retry+1);
    CONSOLE.print("  ");
  }
  return res; // Return the response
}

void BLEConfig::run(){ // Method to run the BLE configuration
  int baud; // Baud rate variable
  bool found = false; // Flag to indicate if BLE module is found
  for (int i=0; i < 12; i++){ // Loop through possible baud rates
    switch(i){ // Switch case for baud rate selection
      case 3: baud=9600; break; // 9600 baud
      case 7: baud=115200; break; // 115200 baud
      default: continue; // Continue for other cases
    }
    CONSOLE.print("trying to detect Bluetooth 4.0/BLE module (make sure your phone is NOT connected)");
    CONSOLE.print(baud);
    CONSOLE.println("...");
    BLE.begin(baud); // Begin BLE communication at selected baud rate
    String res = exec("AT\r\n", false); // Execute AT command
    if (res.indexOf("OK") != -1){ // Check if response is OK
      CONSOLE.println("Bluetooth 4.0/BLE module found!");
      if (baud == BLE_BAUDRATE) {
        found = true;
        break;
      } else {
        exec("AT+BAUD8\r\n", true); // Set baud rate
        BLE.begin(BLE_BAUDRATE); // Begin BLE communication at new baud rate
        found = true;
        break;
      }
    }
  }

  if (found) { // If BLE module found
    exec("AT+VERSION\r\n", true); // Get firmware version
#if defined(BLE_NAME)      
    exec("AT+NAME" BLE_NAME "\r\n", true); // Set BLE name if defined
#endif
    exec("AT+RESET\r\n", true); // Apply new settings and reboot module
    return;
  } else {
    CONSOLE.println("error: no BLE module found!"); // Print error if not found
  }
}
