/*
// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

  SkyTraq Phoenix GNSS receiver binary message protocol parser
  
*/

#ifndef SKYTRAQ_h
#define SKYTRAQ_h

#include "Arduino.h"			
#include "SkyTraqNmeaParser.h"
#include "../../gps.h"
#include "../driver/RobotDriver.h"

class SKYTRAQ : public SkyTraqNotifyFun, public GpsDriver {
  public:
    typedef enum {
        GOT_NONE,
        GOT_SYNC1,
        GOT_SYNC2,
        GOT_CLASS,
        GOT_ID,
        GOT_LENGTH1,
        GOT_LENGTH2, 
        GOT_PAYLOAD,
        GOT_CHK 

    } state_t;        
    SKYTRAQ();    
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
  private:
    // The SkyTraqNmeaParser object
    SkyTraqNmeaParser parser;

    uint32_t _baud;  	
    HardwareSerial* _bus;
    Client* _client;
    state_t state;
    int msgid;
    int msglen;
    int chk;
    int count;
    char payload[2000];                                          
    bool useTCP;
    bool debug;
    bool verbose;
    // The SkyTraqNmeaParser result
    const GnssData* gdata;
    // Notification of SkyTraqNmeaParser
    U32 gnssUpdateFlag;
    unsigned long solutionTimeout;
    
    void begin();
    void addchk(int b);
    void dispatchMessage();
    long unpack_int32(int offset);
    long unpack_int16(int offset);
    long unpack_int8(int offset);
    long unpack(int offset, int size);
    void parseBinary(int b);	 

    virtual bool gnssUpdated(U32 f, const char* buf, ParsingType type);
    bool processNmea(U32 f, const char* buf, ParsingType type); 	    
};

#endif
