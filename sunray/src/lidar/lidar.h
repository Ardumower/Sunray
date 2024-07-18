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

class LidarGpsDriver : public GpsDriver {
  public:
    LidarGpsDriver();    
    void begin();
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
  private:

};


class LidarImuDriver: public ImuDriver {    
  public:    
    LidarImuDriver();    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;        
    bool dataAvail;
};


class LidarBumperDriver: public BumperDriver {
  public:
    LidarBumperDriver();
    void begin() override;
    void run() override;
    bool nearObstacle() override;
    bool obstacle() override;
    bool getLeftBumper() override;
    bool getRightBumper() override;
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
    bool triggerBumper;
    bool triggerNearObstacle;
};


#endif
