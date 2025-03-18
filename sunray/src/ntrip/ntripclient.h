#ifndef NTRIP_CLIENT
#define NTRIP_CLIENT

#ifdef __linux__

#include <BridgeClient.h>
  //#include <WiFiClient.h>
#include <Arduino.h>
#include "base64.h"
#include "../driver/RobotDriver.h"



// TODO: should not extend WiFiClient to make it reusable
class NTRIPClient : public WiFiClient{
  protected:
    GpsDriver *gpsDriver;
    unsigned long reconnectTimeout;
    unsigned long ggaTimeout;
    unsigned long nextGGASendTime;
    unsigned long nextInfoTime;
    int bytesReceived;
    int bytesValid;
    void connectNTRIP();
    bool reqSrcTbl(char* host,int port);   //request MountPoints List serviced the NTRIP Caster 
    bool reqRaw(char* host,int port,char* mntpnt,char* user,char* psw);      //request RAW data from Caster 
    bool reqRaw(char* host,int port,char* mntpnt); //non user
    int readLine(char* buffer,int size);
    void processRTCMData(byte *inputBuffer, int bytesRead);
  public :
    String nmeaGGAMessage = ""; // next NMEA GGA message to send to NTRIP caster/server
    String nmeaGGAMessageSource = ""; // source of NMEA GGA message
    void begin(GpsDriver *aGpsDriver);    
    void run();        
  
};

#endif  // __linux__

#endif
