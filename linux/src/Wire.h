/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <inttypes.h>

#ifdef __cplusplus
extern "C"{
#endif


#ifdef __cplusplus
} // extern "C"
#endif

#include "Stream.h"

#define BUFFER_LENGTH 32

class TwoWire : public Stream {
  private:

    // static?    
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static uint8_t busAddress;
    static int busFd;
    
  public:
    TwoWire();
    void begin();
    void end();
    void begin(uint8_t);
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void){}

    inline size_t write(unsigned long n) { return this->write((uint8_t)n); }
    inline size_t write(long n) { return this->write((uint8_t)n); }
    inline size_t write(unsigned int n) { return this->write((uint8_t)n); }
    inline size_t write(int n) { return this->write((uint8_t)n); }
    using Print::write;
};

extern TwoWire Wire;

#endif

