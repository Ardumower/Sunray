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

void ArduMower::Encrypt::calculateKey() {
  if (password == 0) return;
  if (challenge == 0) return;

  key = password % challenge;
}

void ArduMower::Encrypt::encrypt(char* buffer, size_t len) {
  if (!on || key == 0) return;

  for (int i=0; i < len; i++) {
    const unsigned char b = (unsigned char)buffer[i];
    int c = b;

    c += key;
    if (c > 126) c -= (126 - 31);

    buffer[i] = (char)c;
  }
}

void ArduMower::Encrypt::decrypt(char* buffer, size_t len) {
  if (!on || key == 0) return;

  for (int i = 0; i < len; i++) {
    const unsigned char b = (unsigned char)buffer[i];
    if (b < 32 || b > 126) continue;

    int c = b;

    c -= key;
    if (c <= 31) c += (126 - 31);

    buffer[i] = (char)c;
  }
}

#endif
