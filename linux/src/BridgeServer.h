/*
  Server.h - Server class for Linux
*/

#ifndef pi_server_h
#define pi_server_h

#include "Arduino.h"
#include "Server.h"
#include "BridgeClient.h"

#include <pthread.h>


typedef void(*BridgeServerHandler)(Client&);

class BridgeServer : public Server {
  private:
    int sockfd;
    int pollfd;
    pthread_mutex_t eventsMutex;
    int clientSock;
    int eventsCount;
    pthread_t thread_id;
    struct epoll_event *events;
    uint16_t _port;
    uint8_t _max_clients;
    bool _listening;
    BridgeServerHandler _cb;
    BridgeClient *clients;

    //cleans all disconnected clients
    void cleanup();
    int setSocketOption(int option, char* value, size_t len);
  public:
    void listenOnLocalhost(){}

    BridgeServer(uint16_t port=80, uint8_t max_clients=8):sockfd(-1),pollfd(0),events(NULL),_port(port),_max_clients(max_clients),_listening(false),_cb(NULL),clients(NULL){}
    ~BridgeServer(){ end();}
    BridgeClient available();
    BridgeClient accept(){return available();}
    virtual void begin();
    virtual size_t write(const uint8_t *data, size_t len);
    virtual size_t write(uint8_t data){
      return write(&data, 1);
    }
    using Print::write;

    void end();
    operator bool(){return _listening;}
    int setTimeout(uint32_t seconds);
    BridgeClient *clientByFd(int fd){
      BridgeClient *c = clients;
      while(c != NULL && c->fd() != fd) c = c->next;
      return c;
    }
    void stopAll();
    void run();    
};

#define WiFiEspServer BridgeServer

#endif

