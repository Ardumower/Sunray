// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower bumper driver

#ifndef AM_BUMPER_DRIVER_H
#define AM_BUMPER_DRIVER_H

#include "BumperDriver.h"


class AmBumperDriver: public BumperDriver {
  public:    
    void begin() override;
    void run() override;
    bool obstacle() override;
        
    // get triggered bumper
    void getTriggeredBumper(bool &leftBumper, bool &rightBumper) override;  	  		    
};


#endif
