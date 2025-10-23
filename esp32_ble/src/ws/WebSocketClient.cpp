#include "WebSocketClient.h"

static String base64Encode(const uint8_t* in, size_t len) {
  static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = ((uint32_t)in[i]) << 16;
    if (i + 1 < len) v |= ((uint32_t)in[i + 1]) << 8;
    if (i + 2 < len) v |= ((uint32_t)in[i + 2]);
    out += tbl[(v >> 18) & 0x3F];
    out += tbl[(v >> 12) & 0x3F];
    out += (i + 1 < len) ? tbl[(v >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? tbl[v & 0x3F] : '=';
  }
  return out;
}

bool WebSocketClient::connect() {
  if (_client.connected()) _client.stop();
  if (!_client.connect(_host.c_str(), _port)) {
    Serial.println(F("WS: connect() failed (TLS/TCP)"));
    return false;
  }
  if (!handshake()) {
    Serial.println(F("WS: handshake failed"));
    _client.stop();
    return false;
  }
  _connected = true;
  return true;
}

void WebSocketClient::close() {
  if (_client.connected()) _client.stop();
  _connected = false;
}

bool WebSocketClient::handshake() {
  // Generate random 16-byte key
  uint8_t key[16];
  for (int i = 0; i < 16; i++) key[i] = (uint8_t)random(0, 256);
  String keyB64 = base64Encode(key, sizeof(key));

  // Send HTTP upgrade request
  _client.print(F("GET "));
  _client.print(_path);
  _client.print(F(" HTTP/1.1\r\n"));
  _client.print(F("Host: "));
  _client.print(_host);
  _client.print(F("\r\n"));
  _client.print(F("Upgrade: websocket\r\n"));
  _client.print(F("Connection: Upgrade\r\n"));
  _client.print(F("Sec-WebSocket-Version: 13\r\n"));
  _client.print(F("Sec-WebSocket-Key: "));
  _client.print(keyB64);
  _client.print(F("\r\n\r\n"));

  return readHttpHeaders();
}

bool WebSocketClient::readHttpHeaders() {
  // Wait for HTTP response and parse status line + headers until blank line
  unsigned long timeout = millis() + 5000;
  String line;
  bool first = true;
  bool ok = false;
  while (millis() < timeout) {
    while (_client.available()) {
      int ch = _client.read();
      if (ch < 0) continue;
      if (ch == '\r') continue;
      if (ch == '\n') {
        if (line.length() == 0) {
          return ok; // end of headers
        }
        if (first) {
          first = false;
          if (!line.startsWith("HTTP/1.1 101")) {
            Serial.print(F("WS: bad status: "));
            Serial.println(line);
          } else {
            ok = true;
          }
        }
        line = "";
      } else {
        line += (char)ch;
      }
    }
  }
  return false;
}

bool WebSocketClient::sendText(const String& msg) {
  if (!connected()) return false;
  // Frame header: FIN=1, opcode=1 (text)
  _client.write((uint8_t)0x81);
  // Client-to-server must be masked
  uint8_t maskKey[4];
  for (int i = 0; i < 4; i++) maskKey[i] = (uint8_t)random(0, 256);
  size_t len = msg.length();
  if (len <= 125) {
    _client.write((uint8_t)(0x80 | (uint8_t)len));
  } else if (len <= 0xFFFF) {
    _client.write((uint8_t)0xFE);
    _client.write((uint8_t)((len >> 8) & 0xFF));
    _client.write((uint8_t)(len & 0xFF));
  } else {
    // Very large frames not supported
    return false;
  }
  _client.write(maskKey, 4);
  for (size_t i = 0; i < len; i++) {
    char c = msg[i] ^ maskKey[i % 4];
    _client.write((uint8_t)c);
  }
  return true;
}

bool WebSocketClient::readExact(uint8_t* buf, size_t len, unsigned long timeoutMs) {
  unsigned long t = millis() + timeoutMs;
  size_t got = 0;
  while (got < len && millis() < t) {
    if (_client.available()) {
      int b = _client.read();
      if (b < 0) continue;
      buf[got++] = (uint8_t)b;
    }
  }
  return got == len;
}

bool WebSocketClient::pollText(String& out) {
  out = "";
  if (!connected()) return false;
  if (!_client.available()) return false;

  uint8_t hdr1, hdr2;
  if (!readExact(&hdr1, 1, 50)) return false;
  if (!readExact(&hdr2, 1, 50)) return false;
  uint8_t opcode = hdr1 & 0x0F;
  bool masked = (hdr2 & 0x80) != 0;
  uint64_t len = (hdr2 & 0x7F);
  if (len == 126) {
    uint8_t ext[2];
    if (!readExact(ext, 2, 50)) return false;
    len = ((uint16_t)ext[0] << 8) | (uint16_t)ext[1];
  } else if (len == 127) {
    Serial.println("WS: unsupported header len: 127");
    close();
    return false;
  }

  uint8_t maskKey[4] = {0,0,0,0};
  if (masked) {
    if (!readExact(maskKey, 4, 50)) return false;
  }

  if (opcode == 0x8) { // close
    close();
    return false;
  } else if (opcode == 0x9) { // ping -> respond with pong
    // Consume payload and reply pong with same payload
    String payload;
    for (uint64_t i = 0; i < len; i++) {
      int b = -1;
      unsigned long t = millis() + 50;
      while (b < 0 && millis() < t) { if (_client.available()) b = _client.read(); }
      if (b < 0) break;
      char c = masked ? ((uint8_t)b) ^ maskKey[i % 4] : (char)b;
      payload += c;
    }
    // Send pong
    _client.write((uint8_t)0x8A); // FIN + pong
    _client.write((uint8_t)payload.length());
    for (size_t i = 0; i < payload.length(); i++) _client.write((uint8_t)payload[i]);
    return false;
  } else if (opcode == 0xA) { // pong
    for (uint64_t i = 0; i < len; i++) { int b = _client.read(); (void)b; }
    return false;
  }

  if (opcode != 0x1) { // only text supported
    for (uint64_t i = 0; i < len; i++) { int b = _client.read(); (void)b; }
    return false;
  }
  for (uint64_t i = 0; i < len; i++) {
    int b = -1;
    unsigned long t = millis() + 100;
    while (b < 0 && millis() < t) { if (_client.available()) b = _client.read(); }
    if (b < 0) { Serial.println("WS: timeout, closing"); close(); return false; }
    char c = masked ? ((uint8_t)b) ^ maskKey[i % 4] : (char)b;
    out += c;
  }
  return true;
}
