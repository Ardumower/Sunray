// Ardumower Sunray 

// Linux CAN bus class (with FIFO)

#ifdef __linux__

#ifndef LINUX_CAN_H
#define LINUX_CAN_H

#include <Arduino.h>
#include "can.h"
#include "fifo.h"


#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <pthread.h>


#define CAN_FIFO_FRAMES_RX 2000
#define CAN_FIFO_FRAMES_TX 2000

class LinuxCAN : public CAN
{
  public:
    LinuxCAN();
    virtual bool begin() override;
    virtual bool available() override;
    virtual bool read(can_frame_t &frame) override;  
    virtual bool write(can_frame_t frame) override;
    virtual bool close() override;
    virtual bool runTx();
    virtual bool runRx();
    unsigned long frameCounterRx;
    unsigned long frameCounterTx;    
  private:
    FiFo<can_frame_t, CAN_FIFO_FRAMES_RX> fifoRx;
    FiFo<can_frame_t, CAN_FIFO_FRAMES_RX> fifoTx;
    pthread_t thread_rx_id;
    pthread_t thread_tx_id;
    int sock;
};



#endif

#endif