
// ICM driver 

#ifndef ICM_DRIVER_H
#define ICM_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../icm/ICM_20948.h"


class IcmDriver: public ImuDriver {    
  public:    
    IcmDriver();    
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;        
  protected:
    ICM_20948_I2C icm;
};


#endif
