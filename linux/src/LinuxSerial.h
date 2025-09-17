#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#include "Stream.h"
#include "HardwareSerial.h"
#include "fifo.h"

    
#define LINUX_SERIAL_FIFO_SIZE_RX 20000
#define LINUX_SERIAL_FIFO_SIZE_TX 20000


class LinuxSerial : public HardwareSerial{
  protected:
    int            _stream = 0;
    struct termios _termios;
    String         devPath;
    FiFo<byte, LINUX_SERIAL_FIFO_SIZE_TX> fifoRx;
    FiFo<byte, LINUX_SERIAL_FIFO_SIZE_RX> fifoTx;
    pthread_t thread_rx_id = 0;
    pthread_t thread_tx_id = 0; 
    bool open(const char *devicePath);
    bool setBaudrate(uint32_t baudrate);
    unsigned long frameCounterRx;
    unsigned long frameCounterTx;    
  public:
    LinuxSerial() { _stream = 0; };
    LinuxSerial(const char *devicePath){
      begin(devicePath);
    }
    LinuxSerial(const char *devicePath, uint32_t baudrate){
      begin(devicePath, baudrate);      
    }
    virtual ~LinuxSerial() { end(); };
    virtual void begin(const char *devicePath, uint32_t baudrate);    
    virtual void begin(const char *devicePath);        
    virtual void begin(uint32_t baudrate) override;     
    virtual void end() override;

    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;

    virtual size_t write(const uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;

    virtual bool runTx();
    virtual bool runRx();

    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }

  };

#endif
