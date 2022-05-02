// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// pin manager
// replacement for Arduino wiring, allowing us to change PWM frequency
// and fixes Arduino built-in Due analogWrite to work properly on all pins

#ifndef PINMAN_H
#define PINMAN_H

#include <Arduino.h>


#define PWM_FREQ_3900    0
#define PWM_FREQ_29300   1


class PinManager {
  public:  
    void begin();
	  void analogWrite( uint32_t ulPin, uint32_t ulValue, byte pwmFreq ) ;  
		void setDebounce(int pin, int usecs);  // reject spikes shorter than usecs on pin
};



#endif 
