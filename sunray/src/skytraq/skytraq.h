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

class SKYTRAQ : public SkyTraqNotifyFun{
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
    
    unsigned long iTOW;
    int numSV;         // #signals tracked 
    int numSVdgps;     // #signals tracked with DGPS signal
    double lon;        // deg
    double lat;        // deg
    double height;     // m
    float relPosN;     // m
    float relPosE;     // m
    float relPosD;     // m
    float heading;     // rad
    float groundSpeed; // m/s
    float accuracy;    // m
    float hAccuracy;   // m
    float vAccuracy;   // m
    SolType solution;    
    bool solutionAvail;
    unsigned long dgpsAge;
    unsigned long chksumErrorCounter;
    unsigned long dgpsChecksumErrorCounter;
    unsigned long dgpsPacketCounter;    
    
    SKYTRAQ();
    void begin(HardwareSerial& bus,uint32_t baud);
    void run();
    bool configure();  
    void reboot();
  private:
    // The SkyTraqNmeaParser object
    SkyTraqNmeaParser parser;

    uint32_t _baud;  	
    HardwareSerial* _bus;
    state_t state;
    int msgid;
    int msglen;
    int chk;
    int count;
    char payload[2000];                                          
    bool debug;
    bool verbose;
    // The SkyTraqNmeaParser result
    const GnssData* gdata;
    // Notification of SkyTraqNmeaParser
    U32 gnssUpdateFlag;
    
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
