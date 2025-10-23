// Minimal WebSocket (RFC6455) client using Arduino Client-compatible transport
#ifndef ESP32_WS_WEBSOCKETCLIENT_H
#define ESP32_WS_WEBSOCKETCLIENT_H

#include <Arduino.h>
#include <Client.h>

class WebSocketClient {
public:
  WebSocketClient(Client& c, const String& host, uint16_t port, const String& path)
    : _client(c), _host(host), _port(port), _path(path), _connected(false) {}

  bool connect();
  void close();
  bool connected() const { return _connected && _client.connected(); }

  // Send a text message (client frames must be masked)
  bool sendText(const String& msg);

  // Receive next text message into out (non-blocking, returns true if a full message was read)
  bool pollText(String& out);

private:
  bool handshake();
  bool readHttpHeaders();
  bool readExact(uint8_t* buf, size_t len, unsigned long timeoutMs);

  Client& _client;
  String _host;
  uint16_t _port;
  String _path;
  bool _connected;
};

#endif

