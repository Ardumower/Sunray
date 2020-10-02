// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
#if defined(_SAM3XA_)
  #include "src/due/DueTimer.h"
#else
  #include "src/agcm4/Adafruit_ZeroTimer.h"    // __SAMD51__
#endif



volatile boolean tone_pin_state = false;

void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state);  
}


#if defined(__SAMD51__)
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}
#endif 



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
   pinMode(pinBuzzer, OUTPUT);                
   digitalWrite(pinBuzzer, LOW);
   toneIdx=0;
   nextToneTime=0;   
}


void Buzzer::tone( uint16_t  freq )
{
#if defined(_SAM3XA_)
  pinMode(pinBuzzer, OUTPUT);
  Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();   
#else      // __SAMD51__
  //::tone(pinBuzzer, freq);    

  // Set up the flexible divider/compare
  uint8_t divider  = 1;
  uint16_t compare = 0;
  tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
  
  divider = 16;
  prescaler = TC_CLOCK_PRESCALER_DIV16;
  compare = (48000000/16)/freq;   
  
  zerotimer.enable(false);
  zerotimer.configure(prescaler,       // prescaler
          TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
          TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
          );

  zerotimer.setCompare(0, compare);
  zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, toneHandler);
  zerotimer.enable(true);
#endif     
}


void Buzzer::noTone(){
#if defined(_SAM3XA_)
  Timer1.stop();  
  digitalWrite(pinBuzzer, LOW);
#else  // __SAMD51__
  //::noTone(pinBuzzer);     
  zerotimer.enable(false);
  digitalWrite(pinBuzzer, LOW);
#endif     
}

