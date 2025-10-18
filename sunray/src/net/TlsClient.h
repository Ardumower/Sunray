// Cross-platform TLS client for Arduino Client API
#ifndef NET_TLSCLIENT_H
#define NET_TLSCLIENT_H

#include <Arduino.h>
#include <Client.h>

#ifdef __linux__
  #include "BridgeSecureClient.h"
  class TlsClient : public BridgeSecureClient {
  public:
    TlsClient(const char* caPath, const char* certPath, const char* keyPath, const char* serverName)
      : BridgeSecureClient(caPath, certPath, keyPath, serverName) {}
  };
#else
  // For MCU targets, require WiFiClientSecure
  #ifndef USE_WIFI_CLIENT_SECURE
    #error "Define USE_WIFI_CLIENT_SECURE and include WiFiClientSecure for TLS (mTLS is mandatory for WebSocket)."
  #endif
  #include <WiFiClientSecure.h>

  class TlsClient : public Client {
  public:
    TlsClient(const char* caPem, const char* certPem, const char* keyPem, const char* serverName)
      : _serverName(serverName ? serverName : "") {
      if (caPem && *caPem) _inner.setCACert(caPem);
      if (certPem && *certPem) _inner.setCertificate(certPem);
      if (keyPem && *keyPem) _inner.setPrivateKey(keyPem);
      if (_serverName.length()) _inner.setHostname(_serverName.c_str());
    }

    int connect(IPAddress ip, uint16_t port) override { return _inner.connect(ip, port); }
    int connect(const char *host, uint16_t port) override { return _inner.connect(host, port); }
    size_t write(uint8_t data) override { return _inner.write(data); }
    size_t write(const uint8_t *buf, size_t size) override { return _inner.write(buf, size); }
    int available() override { return _inner.available(); }
    int read() override { return _inner.read(); }
    int read(uint8_t *buf, size_t size) override { return _inner.read(buf, size); }
    int peek() override { return _inner.peek(); }
    void flush() override { return _inner.flush(); }
    void stop() override { return _inner.stop(); }
    uint8_t connected() override { return _inner.connected(); }
    operator bool() override { return (bool)_inner; }

    using Print::write;

  private:
    WiFiClientSecure _inner;
    String _serverName;
  };
#endif

#endif // NET_TLSCLIENT_H
