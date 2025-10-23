// Cloud WebSocket client (ESP32) to connect robot UART to Sunray cloud
// Similar to linux/httpserver.cpp::processWifiWSClient(), but using ArduinoWebsockets

#include "config.h"

#ifdef USE_CLOUD

#include <Arduino.h>
#include <WiFi.h>
// Use ESP32 core HTTPS (mbedTLS) with CA bundle, and a minimal WS client over WiFiClientSecure
#include <WiFiClientSecure.h>
#include <time.h>
#include <esp_crt_bundle.h>
// Extern symbols provided by the ESP32 core certificate bundle.
// Different core versions/targets expose different names. Declare them weak and choose at runtime.
extern const uint8_t x509_crt_bundle_start[] asm("_binary_x509_crt_bundle_start") __attribute__((weak));
extern const uint8_t x509_crt_bundle_end[]   asm("_binary_x509_crt_bundle_end")   __attribute__((weak));
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_rootca_crt_bundle_start") __attribute__((weak));
extern const uint8_t rootca_crt_bundle_end[]   asm("_binary_rootca_crt_bundle_end")   __attribute__((weak));
extern const uint8_t ca_cert_bundle_start[] asm("_binary_ca_cert_bundle_start") __attribute__((weak));
extern const uint8_t ca_cert_bundle_end[]   asm("_binary_ca_cert_bundle_end")   __attribute__((weak));
#include "src/ws/WebSocketClient.h"

// Backoff for reconnect attempts
static Backoff cloudBackoff(1000, 20000, 1.4);
static bool cloudConnected = false;
static unsigned long wsLastRxTime = 0;
static unsigned long wsNextConnectTime = 0;
static WiFiClientSecure cloudTls;
static WebSocketClient* ws = nullptr;

bool cloud_is_connected() {
  return cloudConnected;
}

// Helper: build ws/wss URL like Linux version
static String cloud_url() {
  String path;
  path += String("/ws/robot?connect_key=") + String(WS_ROBOT_CONNECT_KEY) + String("&proto=at");
  return path;
}

static void cloud_setup_tls() {
  // Strictly use ESP32 core CA bundle (no fallback)
  // Arduino-ESP32 exposes bundle as binary symbols; pass pointer and size
  const uint8_t* start = nullptr;
  const uint8_t* end   = nullptr;
  if ((intptr_t)&x509_crt_bundle_start != 0 && (intptr_t)&x509_crt_bundle_end != 0) {
    start = x509_crt_bundle_start; end = x509_crt_bundle_end;
  } else if ((intptr_t)&rootca_crt_bundle_start != 0 && (intptr_t)&rootca_crt_bundle_end != 0) {
    start = rootca_crt_bundle_start; end = rootca_crt_bundle_end;
  } else if ((intptr_t)&ca_cert_bundle_start != 0 && (intptr_t)&ca_cert_bundle_end != 0) {
    start = ca_cert_bundle_start; end = ca_cert_bundle_end;
  }
  if (!start || !end) {
    CONSOLE.println("WS: ERROR no CA bundle symbols found in core");
    // Intentionally do not fallback to insecure or pinned CA per requirement
    return;
  }  else {
    CONSOLE.println("WS: CA bundle symbols found in core");
  }
  size_t bundle_size = (size_t)(end - start);
  cloudTls.setCACertBundle(start, bundle_size);
  cloudTls.setHandshakeTimeout(15000);
  cloudTls.setTimeout(15000);
  static const char* alpn_protos[] = { "http/1.1", 0 };
  cloudTls.setAlpnProtocols(alpn_protos);
}

void cloud_setup() {
  cloud_setup_tls();
}

static bool cloud_loopConnection() {
  if (cloudConnected) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  // Wait for system time before TLS
  time_t nowsec = time(nullptr);
  if (nowsec < 1609459200) { // 2021-01-01
    CONSOLE.println("WS: waiting for time sync");
    return false;
  }

  const uint32_t now = millis();
  if (now < wsNextConnectTime) return false;
  wsNextConnectTime = now + cloudBackoff.next();

  String path = cloud_url();
  String masked = path;
  int idx = masked.indexOf("connect_key=");
  //if (idx >= 0) {
  //  int start = idx + 12;
  //  int end = masked.indexOf('&', start);
  //  masked = masked.substring(0, start) + String("***") + (end >= 0 ? masked.substring(end) : String(""));
  //}
  CONSOLE.print("WS: connecting wss://");
  CONSOLE.print(WS_HOST);
  CONSOLE.print(":");
  CONSOLE.print(WS_PORT);
  CONSOLE.print(" path=");
  CONSOLE.println(path);
  IPAddress ip;
  if (WiFi.hostByName(WS_HOST, ip)) {
    CONSOLE.print("WS: resolved ");
    CONSOLE.println(ip);
  } else {
    CONSOLE.println("WS: DNS resolution failed");
  }

  if (ws) { delete ws; ws = nullptr; }
  ws = new WebSocketClient(cloudTls, WS_HOST, WS_PORT, path);
  if (ws->connect()) {
    CONSOLE.println("WS: connected");
    cloudConnected = true;
    cloudBackoff.reset();
    wsLastRxTime = millis();
  } else {
    CONSOLE.print("WS: connect failed (WiFiStatus=");
    CONSOLE.print((int)WiFi.status());
    CONSOLE.print(", localIP=");
    CONSOLE.print(WiFi.localIP());
    CONSOLE.println(")");
  }
  return false;
}

void cloud_loop() {
  if (!cloud_loopConnection()) return;

  // Poll for incoming frames and process AT commands
  if (ws && ws->connected()) {
    String msg;
    while (ws->pollText(msg)) {
      wsLastRxTime = millis();
      if (msg.length() == 0) break;
      while (msg.endsWith("\r") || msg.endsWith("\n")) msg.remove(msg.length()-1);
      CONSOLE.print("cloud rx:");
      CONSOLE.println(msg);
      UART.print(msg);

      char buffer[1024];
      size_t len = 0;
      const uint32_t timeout = millis() + 1000;
      while (millis() < timeout) {
        if (!UART.available()) { delay(1); continue; }
        int ch = UART.read();
        if (ch < 0) { delay(1); continue; }
        buffer[len++] = (char)ch;
        if ((char)ch == '\n') break;
        if ((len + 1) >= sizeof(buffer)) break;
      }
      buffer[len] = 0;
      String out = String(buffer);
      if (out.length()) {
        CONSOLE.print("UART tx:");
        CONSOLE.print(out);
        ws->sendText(out);
      }
    }

    // Reconnect if idle for too long (15s)
    if (millis() - wsLastRxTime > 15000) {
      CONSOLE.println("WS: no RX for 15s, reconnecting");
      ws->close();
      cloudConnected = false;
      wsNextConnectTime = millis() + 2000;
    }
  }
}

#endif // USE_CLOUD
