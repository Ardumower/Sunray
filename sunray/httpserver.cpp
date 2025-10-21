#include "httpserver.h"
#include "config.h"
#include "robot.h"
#include "StateEstimator.h"
#include "LineTracker.h"
#include "Stats.h"
#include "src/op/op.h"
#include "reset.h"
#include "comm.h"

#include "timetable.h"


/* machine-id WS handshake helpers removed; firmware embeds ID in AT+V response */

#ifndef ENABLE_WS_CLIENT
#define ENABLE_WS_CLIENT false
#define WS_HOST ""
#define WS_PORT 8080
#define WS_ROBOT_ID 1
#define WS_ROBOT_CONNECT_KEY "01234567890123456789"
#endif

// TLS defaults (Linux only; non-Linux uses BLE/serial, not WS)
#ifdef __linux__
  #ifndef WS_TLS_ROOT_CA_PATH
  #define WS_TLS_ROOT_CA_PATH "./certs/rootCA.pem"
  #endif
  #ifndef WS_TLS_CLIENT_CERT_PATH
  #define WS_TLS_CLIENT_CERT_PATH "./certs/client.crt"
  #endif
  #ifndef WS_TLS_CLIENT_KEY_PATH
  #define WS_TLS_CLIENT_KEY_PATH "./certs/client.key"
  #endif
  #ifndef WS_TLS_SERVER_NAME
  #define WS_TLS_SERVER_NAME WS_HOST
  #endif
#endif

HttpServer::HttpServer()
  : buf(8),
    server(80),
#ifdef __linux__
  #if WS_USE_TLS
    wsTcp(WS_TLS_ROOT_CA_PATH, WS_TLS_CLIENT_CERT_PATH, WS_TLS_CLIENT_KEY_PATH, WS_TLS_SERVER_NAME),
  #else
    wsTcp(),
  #endif
#else
    wsTcp(),
#endif
    wsClient(wsTcp,
             WS_HOST,
             WS_PORT,
             // Build WS path using connect_key only (no legacy id+secret)
             ([](){
               #ifndef WS_ROBOT_CONNECT_KEY
               #error "Define WS_ROBOT_CONNECT_KEY (connect key) in your config."
               #endif
               return String("/ws/robot?connect_key=") + String(WS_ROBOT_CONNECT_KEY) + String("&proto=at");
             })())
{}

void HttpServer::begin(){
  //server.listenOnLocalhost(); // optional
  server.begin();
}


// process WIFI input (relay client)
// a relay server allows to access the robot via the Internet by transferring data from app to robot and vice versa
// client (app) --->  relay server  <--- client (robot)
void HttpServer::processWifiRelayClient(){
  if (!stateEstimator.wifiFound) return;
  if (!ENABLE_RELAY) return;
  if (!wifiClient.connected() || (wifiClient.available() == 0)){
    if (millis() > nextWifiClientCheckTime){   
      wifiClient.stop();
      CONSOLE.println("WIF: connecting..." RELAY_HOST);    
      if (!wifiClient.connect(RELAY_HOST, RELAY_PORT)) {
        CONSOLE.println("WIF: connection failed");
        nextWifiClientCheckTime = millis() + 10000;
        return;
      }
      CONSOLE.println("WIF: connected!");   
      String s = "GET / HTTP/1.1\r\n";
      s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";        
      s += String(RELAY_PORT) + "\r\n";
      s += "Content-Length: 0\r\n";
      s += "\r\n\r\n";
      wifiClient.print(s);
    } else return;
  }
  nextWifiClientCheckTime = millis() + 10000;     
  
  buf.init();                               // initialize the circular buffer   
  unsigned long timeout = millis() + 500;
    
  while (millis() < timeout) {              // loop while the client's connected    
    if (wifiClient.available()) {               // if there's bytes to read from the client,        
      char c = wifiClient.read();               // read a byte, then
      timeout = millis() + 200;
      buf.push(c);                          // push it to the ring buffer
      // you got two newline characters in a row
      // that's the end of the HTTP request, so send a response
      if (buf.endsWith("\r\n\r\n")) {
        String httpCmd = "";
        while ((wifiClient.connected()) && (wifiClient.available()) && (millis() < timeout)) {
          char ch = wifiClient.read();
          timeout = millis() + 200;
          httpCmd = httpCmd + ch;
          gps.run();
        }
        CONSOLE.print("WIF:");
        CONSOLE.println(httpCmd);
        if (wifiClient.connected()) {
          comm.setCmd(httpCmd);
          comm.processCmd("WIF", true,true,true);
          String s = "HTTP/1.1 200 OK\r\n";
            s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";        
            s += String(RELAY_PORT) + "\r\n";
            s += "Access-Control-Allow-Origin: *\r\n";              
            s += "Content-Type: text/html\r\n";              
            s += "Connection: close\r\n";  // the connection will be closed after completion of the response
            // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                                    
            s += "Content-length: ";
            s += String(comm.getCmdResponse().length());
            s += "\r\n\r\n";  
            s += comm.getCmdResponse();                      
            wifiClient.print(s);                                   
        }
        break;
      }
    }
  }
}



// process WIFI input (App server)
// client (app) --->  server (robot)
void HttpServer::processWifiAppServer()
{
  if (!stateEstimator.wifiFound) return;
  if (!ENABLE_SERVER) return;
  if (wifiLastClientAvailableWait != 0){
    if (millis() < wifiLastClientAvailableWait) return;
    wifiLastClientAvailableWait = 0;
  }

  // listen for incoming clients    
  if (client){
    if (stopClientTime != 0) {
      if (millis() > stopClientTime){
        #ifdef VERBOSE 
          CONSOLE.println("app stopping client");
        #endif
        client.stop();
        stopClientTime = 0;                   
      }
      return;    
    }     
  }
  if (!client){
    //CONSOLE.println("client is NULL");
    client = server.available();      
  }
  if (client) {                               // if you get a client,
    #ifdef VERBOSE
      CONSOLE.println("New client");             // print a message out the serial port
    #endif
    battery.resetIdle();
    buf.init();                               // initialize the circular buffer  
    if (client.available() != wifiLastClientAvailable) {
      wifiLastClientAvailable = client.available();
      wifiLastClientAvailableWait = millis() + 50;
      return;
    }
    unsigned long timeout = millis() + 50;
    unsigned long httpStartTime = millis(); 
    while ( (client.connected()) && (millis() < timeout) ) {              // loop while the client's connected
      if (client.available()) {               // if there's bytes to read from the client,        
        char c = client.read();               // read a byte, then
        timeout = millis() + 50;
        buf.push(c);                          // push it to the ring buffer
        // you got two newline characters in a row
        // that's the end of the HTTP request, so send a response
        if (buf.endsWith("\r\n\r\n")) {
          String httpCmd = "";
          while ((client.connected()) && (client.available()) && (millis() < timeout)) {
            char ch = client.read();
            timeout = millis() + 50;
            httpCmd = httpCmd + ch;
            gps.run();
          }
          if (millis() > wifiVerboseStopTime){
            wifiVerboseStopTime = 0;
          }    
          if (wifiVerboseStopTime != 0){          
            CONSOLE.print("WIF:");
            CONSOLE.println(httpCmd);            
          }
          if (client.connected()) {
            comm.setCmd(httpCmd);
            comm.processCmd("WIF",true,true, (wifiVerboseStopTime != 0) );
            client.print(
              "HTTP/1.1 200 OK\r\n"
              "Access-Control-Allow-Origin: *\r\n"              
              "Content-Type: text/html\r\n"              
              "Connection: close\r\n"  // the connection will be closed after completion of the response
              // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                        
              );
            client.print("Content-length: ");
            client.print(comm.getCmdResponse().length());
            client.print("\r\n\r\n");                        
            client.print(comm.getCmdResponse());                                   
          }
          break;
        }
      } 
      gps.run();
    }    
    // give the web browser time to receive the data
    stopClientTime = millis() + 100;
    unsigned long httpEndTime = millis();    
    int httpDuration = httpEndTime - httpStartTime;
    if (httpDuration > 500){
      wifiVerboseStopTime = millis() + 30000;
      CONSOLE.print("HTTP WARN: high server duration: ");
      CONSOLE.println(httpDuration);
    }
    //delay(10);
    // close the connection
    //client.stop();
    //CONSOLE.println("Client disconnected");
  }                  
}

// process WIFI input (WebSocket robot gateway client)
// robot (this) ---> WebSocket server (cloud backend)
void HttpServer::processWifiWSClient() {
  if (!stateEstimator.wifiFound) return;
  if (!ENABLE_WS_CLIENT) return;

  // Maintain connection
  if (!wsClient.connected()) {
    if (millis() < wsNextConnectTime) return;
  #if defined(__linux__) && WS_USE_TLS
      CONSOLE.print("WS: connecting wss://");
  #else
      CONSOLE.print("WS: connecting ws://");
  #endif
    CONSOLE.print(WS_HOST);
    CONSOLE.print(":");
    CONSOLE.print(WS_PORT);
    // Show request path (mask connect_key)
    {
      String path = String("/ws/robot?connect_key=") + String(WS_ROBOT_CONNECT_KEY) + String("&proto=at");
      int idx = path.indexOf("connect_key=");
      if (idx >= 0) {
        int start = idx + 12;
        int end = path.indexOf('&', start);
        String masked = path.substring(0, start) + String("***");
        if (end >= 0) masked += path.substring(end);
        path = masked;
      }
      CONSOLE.print(" path=");
      CONSOLE.print(path);
    }
    CONSOLE.println(" ...");
    if (!wsClient.connect()) {
      CONSOLE.println("WS: connect failed");
      wsNextConnectTime = millis() + 10000;
      return;
    }
    CONSOLE.println("WS: connected");
    wsLastRxTime = millis();
  }

  // No periodic telemetry push; robot answers requests from gateway

  // Poll for incoming frames and process AT commands (may be encrypted ASCII)
  String msg;
  while (wsClient.connected() && wsClient.pollText(msg)) {
    wsLastRxTime = millis();
    if (msg.length() == 0) break;
    // Trim CRLF if present
    while (msg.endsWith("\r") || msg.endsWith("\n")) msg.remove(msg.length()-1);
    // Pass through to existing command handler with decrypt=true
    comm.setCmd(msg);
    comm.processCmd("WS", true, true, false);
    if (comm.getCmdResponse().length() > 0) {
      wsClient.sendText(comm.getCmdResponse());
    }
    battery.resetIdle();    
  }

  // If no inbound frames received for 15s, force reconnect
  if (wsClient.connected()) {
    if (millis() - wsLastRxTime > 15000) {
      CONSOLE.println("WS: no RX for 15s, reconnecting");
      wsClient.close();
      wsNextConnectTime = millis() + 2000;
    }
  }
  // If connection was closed by server (handled in pollText), pause before reconnecting
  if (!wsClient.connected()) {
    if (wsNextConnectTime < millis() + 2000) wsNextConnectTime = millis() + 2000;
  }
}
