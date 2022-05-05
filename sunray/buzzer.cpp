// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
#include "robot.h"


void Buzzer::sound(SoundSelect idx, bool async){
  soundIdx = idx;
  toneIdx = 0;
  nextToneTime = millis();
  if (!async){
    while (nextToneTime != 0){
      run();
    }
  }
}

bool Buzzer::isPlaying(){
  return (nextToneTime != 0);
}

void Buzzer::run(){  
  if (nextToneTime == 0) return;
  unsigned long m = millis();
  if (m < nextToneTime) return;
  switch (soundIdx){
    case SND_READY:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 100; break;
        case 1: noTone();  nextToneTime = m + 100; break;
        case 2:            nextToneTime = 0;       break;
      }
      break;
    case SND_PROGRESS:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 20;  break;
        case 1: noTone();  nextToneTime = m + 20;  break;
        case 2:         	 nextToneTime = 0;      break;
      }
      break;
    case SND_OVERCURRENT:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 50;  break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(4200); nextToneTime = m + 50;  break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4:         	 nextToneTime = 0;       break;
      }
      break;
    case SND_WARNING:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 200;  break;
        case 1: noTone();  nextToneTime = m + 2000; break;
        case 2: tone(4200); nextToneTime = m + 200;  break;
        case 3: noTone();  nextToneTime = m + 2000; break;
				case 4: tone(4200); nextToneTime = m + 200;  break;
				case 5: noTone();  nextToneTime = m + 2000; break;
        case 6:            nextToneTime = 0;       break;
      }
      break;			
    case SND_TILT:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 100; break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(4200); nextToneTime = m + 100; break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4:         	 nextToneTime = 0;       break;
      }
      break;
    case SND_ERROR:
      switch (toneIdx){
        case 0: tone(4200); nextToneTime = m + 500; break;
        case 1: noTone();  nextToneTime = m + 200; break;
        case 2: tone(4200); nextToneTime = m + 500; break;
        case 3: noTone();  nextToneTime = m + 200; break;
        case 4:            nextToneTime = 0;       break;
      }
      break;      
  }
  toneIdx++;
}

void Buzzer::begin()
{
  buzzerDriver.begin();
  toneIdx=0;
  nextToneTime=0;   
}


void Buzzer::tone( uint16_t  freq ){
  #ifdef BUZZER_ENABLE
    buzzerDriver.tone(freq);
  #endif
}


void Buzzer::noTone(){
  #ifdef BUZZER_ENABLE  
    buzzerDriver.noTone();
  #endif
}


