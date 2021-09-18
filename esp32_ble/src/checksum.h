#ifndef _CHECKSUM_H
#define _CHECKSUM_H

#include <string.h>

namespace ArduMower
{
  class Checksum
  {
  private:
    unsigned char val;
  public:
    Checksum() : val(0) {}

    void update(char c) { val += c; };

    void update(String& s) {
      for(int i=0; i < s.length(); i++) {
        update(s[i]);
      }
    }

    unsigned char value() { return val; }
  };  
}

#endif
