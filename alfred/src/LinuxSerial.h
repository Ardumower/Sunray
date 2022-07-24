#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stdint.h>
#include <unistd.h>
#include <termios.h>

#include "Stream.h"
#include "HardwareSerial.h"
    
#define SERIAL_BUF_SZ 8192

class LinuxSerial : public HardwareSerial{
  protected:
    int            _stream;
    struct termios _termios;
    String         devPath;
    bool open(const char *devicePath);
    bool setBaudrate(uint32_t baudrate);
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

    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }
};

#endif
