#ifndef LINUX_CONSOLE_H
#define LINUX_CONSOLE_H

#include <stdint.h>
#include <unistd.h>

#include "Stream.h"


class LinuxConsole : public Stream{
  public:
    LinuxConsole() {};
    virtual ~LinuxConsole() {};
    virtual bool begin();
    virtual bool begin(int baudrate);
    virtual void end();

    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    virtual size_t write(const uint8_t c) override;
    
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }
  protected:
    int keyboard;
};

extern LinuxConsole Console;

#define Serial Console

#endif
