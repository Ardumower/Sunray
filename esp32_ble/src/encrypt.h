#ifndef _ENCODE_H
#define _ENCODE_H

#include <stddef.h>
#include <inttypes.h>
#include <stdio.h>

namespace ArduMower
{
  class Encrypt
  {
  private:
    bool on;
    int password;
    int challenge;
    int key;

    void calculateKey();
  public:
    Encrypt() : on(false), password(0), challenge(0), key(0) {}

    void encrypt(char* buffer, size_t len);
    void decrypt(char* buffer, size_t len);

    void setOn(bool enabled) {
      on = enabled;
    }

    void setPassword(int p) { 
      password = p;
      calculateKey();
    }

    void setChallenge(int c) {
      challenge = c;
      calculateKey();
    }
  };
  
}

#endif
