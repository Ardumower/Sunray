// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// bumper sensor - options: 
// 1) bumperduino: https://wiki.ardumower.de/index.php?title=Bumper_sensor
// 2) freewheel:  https://wiki.ardumower.de/index.php?title=Free_wheel_sensor

#ifndef BUMPER_H
#define BUMPER_H



class Bumper {
    public:
      void begin();      
      void run();
      bool obstacle();	    
};


#endif

