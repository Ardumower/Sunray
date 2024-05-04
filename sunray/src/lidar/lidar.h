/*
// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

  LiDAR localization
  
*/

#ifndef LiDAR_h
#define LiDAR_h

#include "Arduino.h"			
#include "../../gps.h"
#include "../driver/RobotDriver.h"

class LiDAR : public GpsDriver {
  public:
    LiDAR();    
    void begin();
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
  private:

};

#endif
