// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// BNO055 driver 

#ifndef BNO_DRIVER_H
#define BNO_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../bno/Adafruit_Sensor.h"
#include "../bno/Adafruit_BNO055.h"
#include "../bno/imumaths.h"


class BnoDriver: public ImuDriver {    
  public:    
    BnoDriver();    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;        
  protected:
    unsigned long nextUpdateTime;
    adafruit_bno055_offsets_t calibrationData;
    //Adafruit_BNO055 bno = Adafruit_BNO055(55); // Adafruit BNO055
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // GY-BNO055    
    void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
    void displaySensorStatus(void);
    void displaySensorDetails(void);
    void displayCalStatus(void);
    void readCalibration();
    void selectChip();
};


#endif
