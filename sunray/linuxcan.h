// Ardumower Sunray 

// Linux CAN bus 

#ifdef __linux__

#ifndef LINUX_CAN_H
#define LINUX_CAN_H

#include <Arduino.h>
#include "can.h"


class LinuxCAN : public CAN
{
  public:
    LinuxCAN();
    virtual bool begin() override;
    virtual bool read(can_frame_t &frame) override;  
    virtual bool write(can_frame_t frame) override;
    virtual bool close() override;
  private:
    int sock;
    
};



#endif

#endif