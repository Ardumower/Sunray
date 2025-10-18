// TLS client for Linux using OpenSSL, implementing Arduino Client API
#pragma once

#ifdef __linux__

#include "Arduino.h"
#include "Client.h"
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class BridgeSecureClient : public Client {
 public:
  BridgeSecureClient(const char* caPath, const char* certPath, const char* keyPath, const char* serverName)
    : _ctx(nullptr), _ssl(nullptr), _sock(-1), _connected(false), _caPath(caPath), _certPath(certPath), _keyPath(keyPath), _serverName(serverName) {}
  ~BridgeSecureClient();

  int connect(IPAddress ip, uint16_t port) override;
  int connect(const char *host, uint16_t port) override;
  size_t write(uint8_t data) override;
  size_t write(const uint8_t *buf, size_t size) override;
  int available() override;
  int read() override;
  int read(uint8_t *buf, size_t size) override;
  int peek() override { return -1; }
  void flush() override {}
  void stop() override;
  uint8_t connected() override { return (_connected && _ssl != nullptr); }
  operator bool() override { return connected(); }

  using Print::write;

 private:
  bool tlsConnect(int sock, const char* host);
  SSL_CTX* _ctx;
  SSL* _ssl;
  int _sock;
  bool _connected;
  const char* _caPath;
  const char* _certPath;
  const char* _keyPath;
  const char* _serverName;
};

#endif // __linux__

