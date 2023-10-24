/*
  HardwareSerial.h - Hardware serial library for Wiring
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "Stream.h"

class HardwareSerial : public Stream {
  protected:
    
  public:
    inline HardwareSerial(){}
    virtual void begin(uint32_t baud) {}
    virtual void end() {}
    virtual int available(void) { return 0; }
    virtual int peek(void) { return 0; }
    virtual int read(void) { return 0; }
    virtual void flush(void) {}
    virtual size_t write(uint8_t) { return 0; }
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }
};

#ifdef SERIAL_TO_CONSOLE
//extern HardwareSerial Serial1;
#else
//extern HardwareSerial Serial;
#endif

#endif
