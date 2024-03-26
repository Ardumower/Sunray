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

class CAN
{
  public:
    virtual bool begin() = 0;
    virtual bool available() = 0;  
    virtual bool read(can_frame_t &frame) = 0;  
    virtual bool write(can_frame_t frame) = 0;
    virtual bool close() = 0;
  private:
    
};



#endif

