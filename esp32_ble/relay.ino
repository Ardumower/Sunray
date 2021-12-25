#include "config.h"

#ifndef USE_RELAY
void relay_setup() {}
void relay_loop() {}
#else

#include "trust.h"
#include <ArduinoWebsockets.h>
#include "base64.h"
using namespace websockets;

WebsocketsClient relayClient;
bool relayConnected;
// reaches 60s max in ~5.5 minutes with ~20 attempts
Backoff relayBackoff(1000, 60000, 1.2);

void relay_setup() {
  relayConnected = false;

  relayClient.setCACert(tls_ca_trust);

  relayClient.onMessage([&](WebsocketsMessage msg) {
    if (!msg.isText())
      return;

    relay_handleMessage(msg.c_str(), msg.length());
  });

  relayClient.onEvent([&](WebsocketsEvent event, String data) {
    switch (event) {
      case WebsocketsEvent::ConnectionOpened:
        relayConnected = true;
        break;

      case WebsocketsEvent::ConnectionClosed:
        relayConnected = false;
        break;
    }
  });

  relay_setupCredentials();

  relayClient.addHeader("Ardumower-Relay-Client-Robot-Timeout", String(RELAY_TIMEOUT));
  relayClient.addHeader("Ardumower-Relay-Client-Ping-Interval", String(RELAY_PINGWAIT * 1000));
}

void relay_loop() {
  relayClient.poll();

  if (!relay_loopConnection())
    return;

  relay_loopPing();
}

void relay_handleMessage(const char* fromRelay, const size_t lenFromRelay) {
  CONSOLE.print("relay rx:");
  CONSOLE.write(fromRelay, lenFromRelay);
  UART.write(fromRelay, lenFromRelay);

  char buffer[1024];
  size_t len = 0;

  // default is 1 second timeout for the ArduMower to finish the response with a '\n' character
  const uint32_t timeout = millis() + RELAY_TIMEOUT;
  while (true) {
    if (millis() >= timeout) {
      relay_onTimeout();
      return;
    }

    if (!UART.available()) {
      delay(1);
      continue;
    }

    int ch = UART.read();
    buffer[len++] = ch;

    if (ch == '\n') {
      buffer[len++] = 0;
      relay_respond(buffer);
      return;
    }

    // +1 for the string null terminator byte
    if ((len + 1) >= sizeof(buffer)) {
      relay_onOverflow();
      return;
    }
  }
}

void relay_respond(const char* buffer) {
  CONSOLE.print("UART tx:");
  CONSOLE.println(buffer);
  relayClient.send(buffer);
}

bool relay_loopConnection() {
  static bool wasConnected = true;
  if (relayConnected != wasConnected) {
    wasConnected = relayConnected;
    if (relayConnected) {
      CONSOLE.println("Relay connected");
      relayBackoff.reset();
    } else {
      CONSOLE.println("Relay disconnected");
    }
  }

  if (relayConnected)
    return true;

  static uint32_t nextAttempt = 0;
  const uint32_t now = millis();
  if (now < nextAttempt)
    return false;

  nextAttempt = now + relayBackoff.next();;

  if (relayClient.connect(RELAY_URL)) CONSOLE.println("Relay connection successful");
  else CONSOLE.println("Relay connection failed");

  // do not return true, rely on event callback handler to flip "connection established"
  return false;
}

void relay_loopPing() {
  static uint32_t nextPing = 0;
  const uint32_t now = millis();
  if (now < nextPing)
    return;

  if (!relayClient.ping())
    return;

  nextPing = now + (1000 * RELAY_PINGWAIT);
}

void relay_onTimeout() {
  CONSOLE.println("Relay UART timeout");
  // no response causes timeout error in relay server
}

void relay_onOverflow() {
  CONSOLE.println("Relay UART overflow");
  // there is no way to signal errors as the transport is transparent
  // fallback to timeout error in relay server
}

void relay_setupCredentials() {
  if (strlen(RELAY_USERNAME) == 0 || strlen(RELAY_PASSWORD) == 0)
    return;

  String plain = String(RELAY_USERNAME) + ":" + RELAY_PASSWORD;
  String enc = base64::encode((const uint8_t *) plain.c_str(), plain.length());
  relayClient.addHeader("Authorization", "Basic " + enc);

  Serial.printf("relay_setupCredentials [%s] [%s]\n", plain.c_str(), enc.c_str());
}

#endif
