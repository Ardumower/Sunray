// Ardumower Sunray 

// CAN bus interface 

#ifndef CAN_H
#define CAN_H

#include <Arduino.h>

typedef struct can_frame_t {
        unsigned long can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
        byte    can_dlc; /* frame payload length in byte (0 .. 8) */
        byte    data[8];
        unsigned long idx;     // frame number
        unsigned long secs;    // timestamp seconds
        unsigned long usecs;   // timestamp microseconds
} can_frame_t;

// base class interface (implementation in subclasses)
class CAN
{
  public:
    unsigned long frameCounterTx = 0; 
    unsigned long frameCounterRx = 0; 
    virtual bool begin() { return false; };
    virtual bool available() { return false; };  
    virtual bool read(can_frame_t &frame) { return false; };  
    virtual bool write(can_frame_t frame) { return false; };
    virtual bool close() { return false; };
  private:
    
};



#endif

