#ifndef NTRIP_CLIENT
#define NTRIP_CLIENT

#ifdef __linux__

#include <BridgeClient.h>
  //#include <WiFiClient.h>
#include <Arduino.h>
#include <base64.h>

// TODO: should not extend WiFiClient to make it reusable
class NTRIPClient : public WiFiClient{
  protected:
    unsigned long reconnectTimeout;
    unsigned long ggaTimeout;
    void connectNTRIP();
    bool reqSrcTbl(char* host,int port);   //request MountPoints List serviced the NTRIP Caster 
    bool reqRaw(char* host,int port,char* mntpnt,char* user,char* psw);      //request RAW data from Caster 
    bool reqRaw(char* host,int port,char* mntpnt); //non user
    int readLine(char* buffer,int size);
  public :
    void begin();    
    void run();        
  
};

#endif  // __linux__

#endif
