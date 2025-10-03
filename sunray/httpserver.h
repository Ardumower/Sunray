// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// robot http (+websocket) interface (app server/cloud client)

#ifndef HTTPSERVER_H
#define HTTPSERVER_H

#include <Arduino.h>
#ifdef __linux__
  #include <BridgeClient.h>
  #include <WiFi.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "RingBuffer.h"
#include "src/net/WebSocketClient.h"

class HttpServer {
public:
  HttpServer();
  void processWifiRelayClient();
  void processWifiAppServer();
  void processWifiWSClient();

private:
  // wifi client
  WiFiEspClient wifiClient;
  unsigned long nextWifiClientCheckTime = 0;

  // ring buffer to increase speed and reduce memory allocation
  ERingBuffer buf;
  int reqCount = 0;                // number of requests received
  unsigned long stopClientTime = 0;
  unsigned long wifiVerboseStopTime = 0;
  unsigned long wifiLastClientAvailableWait = 0;
  int wifiLastClientAvailable = 0;

  // WebSocket-based gateway client (AT protocol)
  WiFiEspClient wsTcp;
  WebSocketClient wsClient;
  unsigned long wsNextConnectTime = 0;
  unsigned long wsLastRxTime = 0;
};

#endif
