// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// MPU driver 

#ifndef MPU_DRIVER_H
#define MPU_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../mpu/SparkFunMPU9250-DMP.h"


class MpuDriver: public ImuDriver {    
  public:    
    MpuDriver();    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;        
  protected:
    MPU9250_DMP mpu;    
    void selectChip();
};


#endif
