// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/*
  BLE 
*/

#ifndef BLE_H
#define BLE_H

#include <Arduino.h>

class BLEConfig
{
  public:
    void run();
    String read();
    String exec(String cmd, bool doRetry);
  private:
    
};

void processBLE();


#endif
