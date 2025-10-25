// Cloud WebSocket client (ESP32) to connect robot UART to Sunray cloud
// Similar to linux/httpserver.cpp::processWifiWSClient(), but using ArduinoWebsockets

#include "config.h"

#ifdef USE_CLOUD

#include <Arduino.h>
#include <WiFi.h>
// Use mbedTLS HTTPS client with pinned Let's Encrypt ISRG Root X1 (same as relay)
#include <WiFiClientSecure.h>
#include <time.h>
#include "trust.h"
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
  // Pin Let's Encrypt ISRG Root X1
  cloudTls.setCACert(tls_ca_trust);
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

  // One-time TLS probe to isolate handshake failures from WS logic
  static bool tlsProbed = false;
  static bool tlsProbeOk = false;
  if (!tlsProbed) {
    tlsProbed = true;
    WiFiClientSecure probe;
    // Reuse pinned CA and settings
    probe.setCACert(tls_ca_trust);
    probe.setHandshakeTimeout(15000);
    probe.setTimeout(15000);
    static const char* alpn_protos[] = { "http/1.1", 0 };
    probe.setAlpnProtocols(alpn_protos);
    CONSOLE.print("TLS probe: connecting to ");
    CONSOLE.print(WS_HOST);
    CONSOLE.print(":");
    CONSOLE.println(WS_PORT);
    esp_task_wdt_reset();
    if (probe.connect(WS_HOST, WS_PORT)) {
      CONSOLE.println("TLS probe: success");
      tlsProbeOk = true;
      probe.stop();
    } else {
      CONSOLE.println("TLS probe: FAILED");
      tlsProbeOk = false;
    }
    esp_task_wdt_reset();
    if (!tlsProbeOk) {
      // Skip WS handshake if TLS itself fails; try again later
      return false;
    }
  }

  if (ws) { delete ws; ws = nullptr; }
  ws = new WebSocketClient(cloudTls, WS_HOST, WS_PORT, path);
  esp_task_wdt_reset();
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
  esp_task_wdt_reset();
  return false;
}

void cloud_loop() {
  // If we believe we're connected but the socket isn't, fix state and schedule reconnect
  if (cloudConnected && (!ws || !ws->connected())) {
    CONSOLE.println("WS: transport closed, scheduling reconnect");
    cloudConnected = false;
    wsNextConnectTime = millis() + 2000;
    if (ws) { ws->close(); delete ws; ws = nullptr; }
  }

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
      UART.println(msg);

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

  }

  // Reconnect if idle for too long (15s) even if ws->connected() flipped already
  if (cloudConnected && (millis() - wsLastRxTime > 15000)) {
    CONSOLE.println("WS: no RX for 15s, reconnecting");
    if (ws) ws->close();
    cloudConnected = false;
    wsNextConnectTime = millis() + 2000;
  }
}

#endif // USE_CLOUD
