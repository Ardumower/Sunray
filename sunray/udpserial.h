// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/*
  UDP serial
*/

#ifndef UDPSERIAL_H
#define UDPSERIAL_H

#include <HardwareSerial.h>
#include <Arduino.h>



class UdpSerial: public Stream{
  public:
    virtual void begin(unsigned long baud);
    void beginUDP();
    virtual size_t write(uint8_t);    
    virtual int available();
    virtual int read();
    virtual int peek();    
    virtual void flush();
};

extern UdpSerial udpSerial; // Making it available as udpSerial


#endif
