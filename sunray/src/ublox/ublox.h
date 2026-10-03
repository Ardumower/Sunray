/*
// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

  ublox f9p UBX parser
  uses UBX messages: 
  1) 'UBX-NAV-PTV'        10  (Navigation Position Velocity Time Solution)
  2) 'UBX-NAV-RELPOSNED'   1  (Relative Positioning Information in NED frame)
  3) 'UBX-NAV-HPPOSLLH'    1  (absolute position)
  4) 'UBX-NAV-VELNED'      1  (velocity)
  5) 'UBX-RXM-RTCM'        5  (RTCM messages)
  6) 'UBX-NAV-SIG'        20 (signal information)
  https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

*/

#ifndef UBLOX_h
#define UBLOX_h

#include "Arduino.h"				
#include "../../gps.h"
#include "../driver/RobotDriver.h"

#define UBX_SYNC1  0xB5
#define UBX_SYNC2  0x62



class UBLOX : public GpsDriver {
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
        GOT_CHKA 

    } state_t;        
    UBLOX();
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void send(const uint8_t *buffer, size_t size) override;
    void sendRTCM(const uint8_t *buffer, size_t size) override;
    void run() override;
    bool isConfiguring() const override;
    bool configure() override;  
    void reboot() override;
  private:
    enum ConfigPhase { CFG_DISABLED, CFG_PROBE_TARGET, CFG_PROBE_FALLBACK,
      CFG_SWITCH_BAUD, CFG_PROBE_VERIFY, CFG_VERSION, CFG_APPLY, CFG_PAUSE,
      CFG_RETRY, CFG_READY };
    ConfigPhase configPhase;
    uint8_t configTx[128];
    uint16_t configTxLength, configTxOffset;
    uint8_t configStep, configAttempt;
    uint32_t configDeadline;
    uint32_t lastPacketTime, serialGeneration;
    bool configResponse, configRejected;
    void startConfigPacket(uint8_t cls, uint8_t id);
    void finishConfigPacket();
    void startConfigValues();
    void addConfigValue(uint32_t key, uint32_t value);
    void startConfigProbe(ConfigPhase phase, uint32_t baud);
    void retryConfiguration();
    void runConfiguration();
    void queueConfigStep();
    bool useTCP;
    Client* _client;    
    uint32_t _baud;  	
    HardwareSerial* _bus;
    state_t state;
    int msgid;
    int msgclass;
    int msglen;
    int chka;
    int chkb;
    int count;
    char payload[2000];                                          
    bool debug;
    bool verbose;
    String unparsedMessage;
    unsigned long solutionTimeout;    

    void begin();
    void addchk(int b);
    void dispatchMessage();
    long unpack_int32(int offset);
    long unpack_int16(int offset);
    long unpack_int8(int offset);
    long unpack(int offset, int size);
    void parse(int b);	  	  
    void calcUBXChecksum(uint8_t *data, size_t length, uint8_t *ck_a, uint8_t *ck_b);
};

#endif
