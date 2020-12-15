// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// bumper driver base

#ifndef BUMPER_DRIVER_H
#define BUMPER_DRIVER_H


class BumperDriver {
  public:    
    virtual void begin();
    virtual void run();
    virtual bool obstacle();
    
    // get triggered bumper
    virtual void getTriggeredBumper(bool &leftBumper, bool &rightBumper);  	  		    
};


#endif
