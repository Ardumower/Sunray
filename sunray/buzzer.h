// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)
// buzzer - play beep sounds (async) // Comment explaining the purpose of the file

#ifndef BUZZER_H // Preprocessor directive to prevent the file from being included more than once
#define BUZZER_H // Definition to mark the file as included

#include <inttypes.h> // Include the header file for fixed-size integer types

enum SoundSelect {SND_READY, SND_PROGRESS, SND_OVERCURRENT, SND_TILT, SND_WARNING, SND_ERROR}; // Enumeration for different sound types

class Buzzer { // Class definition for Buzzer
    public:
      void begin(); // Method to initialize the buzzer
      void sound(SoundSelect idx, bool async = true); // Method to play a sound, with an optional asynchronous flag
      void run(); // Method to run the buzzer
      bool isPlaying(); // Method to check if the buzzer is playing
    protected:     
      void tone(uint16_t freq); // Method to play a tone at a given frequency
      void noTone(); // Method to stop playing a tone
      SoundSelect soundIdx; // Member variable to store the current sound index
      int toneIdx; // Member variable to store the current tone index
      unsigned long nextToneTime; // Member variable to store the time for the next tone
};

#endif // End of preprocessor conditional for BUZZER_H
