/// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "buzzer.h" // Include the header file for the buzzer
#include "config.h" // Include the configuration file
#include <Arduino.h> // Include the Arduino library
#if defined(_SAM3XA_)
  #include "src/due/DueTimer.h" // Include DueTimer library if compiling for Arduino Due
#else
  #include "src/agcm4/Adafruit_ZeroTimer.h" // Include Adafruit_ZeroTimer library if not compiling for Arduino Due
#endif

volatile boolean tone_pin_state = false; // Declare a volatile boolean variable to keep track of the tone pin state

void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state); // Toggle the tone pin state
}

#if defined(__SAMD51__)
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3); // Create an instance of Adafruit_ZeroTimer for SAMD51

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3); // Timer handler for SAMD51
}
#endif 

void Buzzer::sound(SoundSelect idx, bool async){ // Method to play a sound
  soundIdx = idx; // Set the sound index
  toneIdx = 0; // Initialize the tone index
  nextToneTime = millis(); // Set the next tone time
  if (!async){ // If not asynchronous
    while (nextToneTime != 0){ // While next tone time is not zero
      run(); // Run the buzzer
    }
  }
}

bool Buzzer::isPlaying(){ // Method to check if the buzzer is playing
  return (nextToneTime != 0); // Return true if next tone time is not zero
}

void Buzzer::run(){ // Method to run the buzzer
  if (nextToneTime == 0) return; // Return if next tone time is zero
  unsigned long m = millis(); // Get the current time in milliseconds
  if (m < nextToneTime) return; // Return if current time is less than next tone time
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
  toneIdx++; // Increment the tone index
}

void Buzzer::begin(){ // Method to initialize the buzzer
   pinMode(pinBuzzer, OUTPUT); // Set the buzzer pin as output                
   digitalWrite(pinBuzzer, LOW); // Set the buzzer pin low
   toneIdx=0; // Initialize the tone index
   nextToneTime=0; // Initialize the next tone time
}

void Buzzer::tone( uint16_t  freq ){ // Method to play a tone with a specific frequency
#ifdef _SAM3XA_
  pinMode(pinBuzzer, OUTPUT); // Set the buzzer pin as output for Arduino Due
  Timer1.attachInterrupt(toneHandler).setFrequency(freq).start(); // Attach interrupt and set frequency for Arduino Due
#elif __SAMD51__ // If compiling for SAMD51
  // Set up the flexible divider/compare
  uint8_t divider  = 1; // Initialize divider
  uint16_t compare = 0; // Initialize compare
  tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1; // Initialize prescaler
  
  divider = 16; // Set divider to 16
  prescaler = TC_CLOCK_PRESCALER_DIV16; // Set prescaler to divide by 16
  compare = (48000000/16)/freq; // Calculate compare value
  
  zerotimer.enable(false); // Disable zero timer
  zerotimer.configure(prescaler, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_PWM); // Configure zero timer

  zerotimer.setCompare(0, compare); // Set compare value
  zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, toneHandler); // Set callback
  zerotimer.enable(true); // Enable zero timer
#endif     
}

void Buzzer::noTone(){ // Method to stop playing a tone
#ifdef _SAM3XA_
  Timer1.stop(); // Stop Timer1 for Arduino Due
  digitalWrite(pinBuzzer, LOW); // Set the buzzer pin low for Arduino Due
#elif __SAMD51__ // If compiling for SAMD51
  zerotimer.enable(false); // Disable zero timer for SAMD51
  digitalWrite(pinBuzzer, LOW); // Set the buzzer pin low for SAMD51
#endif     
}
