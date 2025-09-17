/*
  Client.h - Base class that provides Client
*/

#ifndef pi_client_h
#define pi_client_h

#include "Arduino.h"
#include "Client.h"

class BridgeServer;

class BridgeClient : public Client {
  protected:
    int sockfd;
    bool _connected;
    BridgeServer *bridgeServer;
    struct hostent *server;
  public:
    BridgeClient *next;
    BridgeClient():bridgeServer(NULL), sockfd(-1),_connected(false),next(NULL),server(NULL){}
    BridgeClient(BridgeServer *aServer):bridgeServer(aServer), sockfd(-1),_connected(false),next(NULL),server(NULL){}
    BridgeClient(BridgeServer *aServer, int fd):bridgeServer(aServer), sockfd(fd),_connected(true),next(NULL),server(NULL){
      //Serial.print("new client fd=");
      //Serial.print(sockfd);
      //Serial.print("  connected=");
      //Serial.println(_connected);
    }
    ~BridgeClient();
    virtual int connect(IPAddress ip, uint16_t port);
    virtual int connect(const char *host, uint16_t port);
    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int available();
    virtual int read();
    virtual int read(uint8_t *buf, size_t size);
    virtual int peek(){return 0;}
    virtual void flush(){}
    virtual void stop();
    virtual uint8_t connected();
    virtual operator bool(){ return connected(); }
    virtual bool operator==(const bool value) { return bool() == value; }
    virtual bool operator!=(const bool value) { return bool() != value; }
    virtual bool operator==(const BridgeClient&);
    virtual bool operator!=(const BridgeClient& rhs) { return !this->operator==(rhs); };

    int fd(){return sockfd;}
    IPAddress remoteIP();
    uint16_t remotePort();
    int setSocketOption(int option, char* value, size_t len);
    int setOption(int option, int *value);
    int getOption(int option, int *value);
    int setTimeout(uint32_t seconds);
    int setNoDelay(bool nodelay);
    bool getNoDelay();

    static IPAddress remoteIP(int fd);
    static uint16_t remotePort(int fd);
    
    friend class BridgeServer;
    using Print::write;
};

#define WiFiEspClient BridgeClient
#define WiFiClient BridgeClient

#endif

