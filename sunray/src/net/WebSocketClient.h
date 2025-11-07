// Minimal WebSocket (RFC6455) client for Arduino Client-compatible transports
#ifndef NET_WEBSOCKETCLIENT_H
#define NET_WEBSOCKETCLIENT_H

#include <Arduino.h>
#include <Client.h>
#ifdef __linux__
// Arduino headers may define min/max macros that break C++ headers (e.g., <limits>)
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <mutex>
// Simple aliases to allow conditional locking on Linux
using WsMutex = std::mutex;
using WsLock = std::lock_guard<std::mutex>;
// Restore common Arduino-style min/max macros for code that relies on them
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#else
// On non-Linux targets, camera streaming over WS is not used; provide no-op mutex
class WsMutex { public: void lock() {} void unlock() {} };
class WsLock { public: explicit WsLock(WsMutex&) {} };
#endif

class WebSocketClient {
public:
  WebSocketClient(Client& c, const String& host, uint16_t port, const String& path)
    : _client(c), _host(host), _port(port), _path(path), _connected(false) {}

  bool connect();
  void close();
  bool connected() const;

  // Send a text message (client frames must be masked)
  bool sendText(const String& msg);
  bool sendBinaryRaw(const uint8_t* data, size_t len);

  // Receive next text message into out (non-blocking, returns true if a full message was read)
  bool pollText(String& out);

private:
  bool handshake();
  bool readHttpHeaders();
  bool readExact(uint8_t* buf, size_t len, unsigned long timeoutMs);
  void closeUnlocked();
  bool connectedUnlocked() const { return _connected && _client.connected(); }

  Client& _client;
  String _host;
  uint16_t _port;
  String _path;
  bool _connected;
  mutable WsMutex _mtx;
};

#endif
