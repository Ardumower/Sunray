#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include "Arduino.h"

#include "LinuxSerial.h"


void LinuxSerial::begin(const char *devicePath){    
  open(devicePath);
}

void LinuxSerial::begin(const char *devicePath, uint32_t baudrate){      
  if (!open(devicePath)) return;
  setBaudrate(baudrate); 
}

void LinuxSerial::begin(uint32_t baudrate){
  setBaudrate(baudrate);
} 

  
bool LinuxSerial::setBaudrate(uint32_t baudrate){
  ::printf("setting baudrate %s %d...\n", devPath.c_str(), baudrate);
  struct termios newtermios;  
  tcgetattr(_stream, &_termios);
  memset(&newtermios, 0, sizeof(struct termios));
  int baud;
  switch(baudrate){
      case 50:  baud = B50; break;
      case 110: baud = B110; break;
      case 300: baud = B300; break;
      case 600: baud = B600; break;
      case 1200: baud = B1200; break;
      case 2400: baud = B2400; break;
      case 4800: baud = B4800; break;
      case 9600: baud = B9600; break;
      case 19200: baud = B19200; break;
      case 38400: baud = B38400; break;
      case 56700: baud = B57600; break;
      case 115200: baud = B115200; break;      
      default: baud = B115200; break;
  }
  int databits = CS8;
  int stopbits = 0;
  int parity = 0;
  int protocol = 0;
  newtermios.c_cflag = baud | stopbits | parity | databits | CLOCAL | CREAD | protocol;
  newtermios.c_cc[VMIN] = 1;
  tcflush(_stream, TCIOFLUSH);
  tcsetattr(_stream, TCSANOW, &newtermios);
  tcflush(_stream, TCIOFLUSH);
  fcntl(_stream, F_SETFL, O_NONBLOCK);
  return true;
}

bool LinuxSerial::open(const char *devicePath){    
  struct termios newtermios;
  devPath = devicePath;
  ::printf("opening serial port %s...\n", devicePath);
  if ((_stream = ::open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK)) <= 0)
  { 
    ::printf("could not open serial port %s\n", devicePath);
    return false;
  }
  return true;
}


void LinuxSerial::end(){
  if(_stream)
  {
    /* reset old settings */
    ::printf("closing serial port %s...\n", devPath.c_str());
    tcsetattr(_stream, TCSANOW, &_termios);
    close(_stream);
    _stream = 0;
  }
}

int LinuxSerial::available(){
    int bytes_avail = 0;
    ioctl(_stream, FIONREAD, &bytes_avail);
    return bytes_avail;
}

int LinuxSerial::peek(){
    return 0;
}

int LinuxSerial::read(){
    char buffer = 0;
    size_t size = 1;
    int j = ::read(_stream, &buffer, size);
    if(j < 0)
    {
      if(errno == EAGAIN)
        return 0;
      else
        return buffer;
    }
    return buffer;
}

void LinuxSerial::flush(){
    //console_flush();
}

size_t LinuxSerial::write(uint8_t c){
    size_t size = 1;
    char *buffer = (char*)&c;
    int j = ::write(_stream, buffer, size);    
    if(j < 0)
    {
        if(errno == EAGAIN)
          return 0;
        else
          return j;
    }
    return j;
}

    
size_t LinuxSerial::write(const uint8_t *buffer, size_t size){
    int j = ::write(_stream, buffer, size);    
    if(j < 0)
    {
        if(errno == EAGAIN)
          return 0;
        else
          return j;
    }
    return j;
}




