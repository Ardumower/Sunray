// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// buzzer - play beep sounds (async)

#ifndef BUZZER_H
#define BUZZER_H


#include <inttypes.h>

enum SoundSelect {SND_READY, SND_PROGRESS, SND_OVERCURRENT, SND_TILT, SND_WARNING, SND_ERROR} ;

class Buzzer {
    public:
      void begin();      
      void sound(SoundSelect idx, bool async = true);
      void run();
      bool isPlaying();
    protected:     
      void tone(uint16_t freq);
      void noTone(); 
      SoundSelect soundIdx;
      int toneIdx;
      unsigned long nextToneTime;
};


#endif

