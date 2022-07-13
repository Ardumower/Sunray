/*   ESP32 firmware providing:
     1. BLE UART bridge (GATT server UART)
     2. WiFi UART bridge (HTTP server UART)


  serial protocol:

  ---COMMANDS---                                                ---ANSWER---
                                AT\r\n                          OK\r\n
  request version               AT+VERSION\r\n                  +VERSION=ESP32 firmware V0.1,Bluetooth V4.0 LE\r\n
  change BLE name               AT+NAMEname\r\n                 +NAME=name\r\n
  reset module                  AT+RESET\r\n                    +RESET\r\n
  send BLE test packet          AT+TEST\r\n                     +TEST\r\n
  connect to wifi               AT+WIFImode,ssid,pass\r\n       +WIFI=mode,ssid,pass\r\n
*/

#include "config.h"

#define VERSION "ESP32 firmware V0.4.5,Bluetooth V4.0 LE"

// watch dog timeout (WDT) in seconds
#define WDT_TIMEOUT 60

// configure below IPs if using static IP
IPAddress av_local_IP(WIFI_STATIC_IP_LOCAL);
IPAddress av_gateway(WIFI_STATIC_IP_GW);
IPAddress av_subnet(WIFI_STATIC_IP_SUBNET);
IPAddress av_primaryDNS(WIFI_STATIC_IP_DNS1); //optional
IPAddress av_secondaryDNS(WIFI_STATIC_IP_DNS2); //optional
      
String ssid = WIFI_STA_SSID;
String pass = WIFI_STA_PSK;


#include <WiFi.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>

#ifdef USE_HTTPS
  // Include certificate data 
  #include "src/cert.h"
  #include "src/private_key.h"
  #include <HTTPSServer.hpp>
  #include <SSLCert.hpp>
#else
  #include <HTTPServer.hpp>
#endif
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
// The HTTPS Server comes in a separate namespace. For easier use, include it here.
using namespace httpsserver;

#ifdef USE_BLE
  #ifdef USE_NIM_BLE
    #include "NimBLEDevice.h"
  #else    
    #include <BLEDevice.h>
    #include <BLEServer.h>
    #include <BLEUtils.h>
    #include <BLECharacteristic.h>
    #include <BLE2902.h>
  #endif
#endif

#include <ArduinoOTA.h>
#include <esp_task_wdt.h>

#ifdef USE_MQTT
  #include "src/adapter.h"
  ArduMower::Adapter mower(UART, ENCRYPTION_PASSWORD, ENCRYPTION_ENABLED);  
#endif

String cmd;
unsigned long nextInfoTime = 0;
unsigned long nextPingTime = 0;
unsigned long nextLEDTime = 0;
unsigned long nextWatchDogResetTime = 0;
unsigned long nextWifiConnectTime = 0;
bool ledStateNew = false;
bool ledStateCurr = false;
int simPacketCounter = 0; 

// ---- BLE ---------------------------
String bleName = NAME;
#ifdef USE_BLE
  BLEServer *pServer = NULL;
  BLECharacteristic * pCharacteristic;
#endif

String bleAnswer = "";
unsigned long bleAnswerTimeout = 0;
bool bleConnected = false;
bool oldBleConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID             "0000FFE0-0000-1000-8000-00805F9B34FB" // UART service UUID
#define CHARACTERISTIC_UUID      "0000FFE1-0000-1000-8000-00805F9B34FB"

#define BLE_BUF_SZ 2048

volatile int rxReadPos = 0;
volatile int rxWritePos = 0;
byte rxBuf[BLE_BUF_SZ];

volatile int txReadPos = 0;
volatile int txWritePos = 0;
byte txBuf[BLE_BUF_SZ];

String notifyData;

// ----- wifi --------------------------
#ifdef USE_HTTPS
  // Create an SSL certificate object from the files included above
  SSLCert cert = SSLCert(
    example_crt_DER, example_crt_DER_len,
    example_key_DER, example_key_DER_len
  );
  HTTPSServer * server = NULL;
#else
  HTTPServer * server = NULL;
#endif

//WiFiServer server(80);
//WiFiClient client;
// We declare some handler functions (definition at the end of the file)
void handleRoot(HTTPRequest * req, HTTPResponse * res);
ResourceNode * nodeRoot      = new ResourceNode("/", "POST", &handleRoot);

// ------------------------------- UART -----------------------------------------------------

void uartSend(String s) {
  //if (!bleConnected) return;
  CONSOLE.print(millis());
  CONSOLE.print(" UART tx:");
  CONSOLE.println(s);
  UART.print(s);
  UART.print("\r\n");
  
  #ifdef USE_MQTT
    mower.tx(s);
    String crlf = "\r\n";                     
    mower.tx(crlf);
  #endif               
}

// ------------------------------- BLE -----------------------------------------------------

#ifdef USE_BLE

// send BLE data to BLE client (write data to FIFO)
void bleSend(String s) {
  if (!bleConnected) {
    CONSOLE.println("bleSend ignoring: not connected");
    return;
  }
  CONSOLE.print(millis());
  CONSOLE.print(" BLE tx:");
  CONSOLE.println(s);
  for (int i = 0; i < s.length(); i++) {
    if ( ((txWritePos + 1) % BLE_BUF_SZ) == txReadPos) {
      CONSOLE.println("BLE: txBuf overflow!");
      break;
    }
    txBuf[txWritePos] = s[i];                          // push it to the ring buffer
    txWritePos = (txWritePos + 1) % BLE_BUF_SZ;      
  }  
  bleNotify();
}

// notify BLE client (send next packet from FIFO)
void bleNotify() {  
  if (txReadPos == txWritePos) return;
  notifyData = "";
  while ((txReadPos != txWritePos) && (notifyData.length() < BLE_MTU-5)) {
    char ch = txBuf[txReadPos];
    notifyData += ch;
    txReadPos = (txReadPos + 1) % BLE_BUF_SZ;
  }  
  if (notifyData.length() > 0){    
    pCharacteristic->setValue( (uint8_t*) notifyData.c_str(), notifyData.length() );
    pCharacteristic->notify();         
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  #ifdef USE_NIM_BLE
    virtual void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc){
      CONSOLE.print("onMTUChange ");
      CONSOLE.println(MTU);
    }
  #endif  

  #ifdef USE_NIM_BLE
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {      
  #else
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param ) {
  #endif
      uint16_t peerMTU = 0;
      rxReadPos = rxWritePos = 0;
      txReadPos = txWritePos = 0;
      /** After connection we should change the parameters if we (don't) need fast response times.
          These settings are 150ms interval, 0 latency, 450ms timout.
          Timeout should be a multiple of the interval, minimum is 100ms.
          I find a multiple of 3-5 * the interval works best for quick response/reconnect.
          Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
      */
      // min(1.25ms units),max(1.25ms units),latency(intervals),timeout(10ms units)      
      #ifdef USE_NIM_BLE
        pServer->updateConnParams(desc->conn_handle, BLE_MIN_INTERVAL, BLE_MAX_INTERVAL, BLE_LATENCY, BLE_TIMEOUT);
        peerMTU = pServer->getPeerMTU(desc->conn_handle);
      #else
        uint16_t connId = pServer->getConnId();
        peerMTU = pServer->getPeerMTU(connId);
        pServer->updateConnParams( param->connect.remote_bda, BLE_MIN_INTERVAL, BLE_MAX_INTERVAL, BLE_LATENCY, BLE_TIMEOUT); // 1, 10, 0, 20    
      #endif      
      CONSOLE.println("---------BLE client connected---------");            
      CONSOLE.print("peerMTU=");
      CONSOLE.println(peerMTU);
      bleConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      bleConnected = false;
      CONSOLE.println("---------BLE client disconnected---------");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {    
    
    #ifdef USE_NIM_BLE
      virtual void onStatus(NimBLECharacteristic* pCharacteristic, Status s, int code){
        //CONSOLE.println("onStatus");            
        if (s == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY) {
          //CONSOLE.println("SUCCESS_NOTIFY");
          // notify success => send next BLE packet...        
          bleNotify();
        }
      }
    #endif

    void onStatus(BLECharacteristic* pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code) {
      //CONSOLE.println("onStatus");      
      if (s == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY) {
        //CONSOLE.println("SUCCESS_NOTIFY");
        // notify success => send next BLE packet...        
        bleNotify();
      }
    }
    // BLE data received from BLE client => save to FIFO
    void onWrite(BLECharacteristic *pCharacteristic) {
      //CONSOLE.print("onWrite: ");
      String rxValue(pCharacteristic->getValue().c_str());
      for (int i = 0; i < rxValue.length(); i++) {
        if ( ((rxWritePos + 1) % BLE_BUF_SZ) == rxReadPos) {
          CONSOLE.println("BLE: rxBuf overflow!");
          break;
        }
        rxBuf[rxWritePos] = rxValue[i];                          // push it to the ring buffer
        rxWritePos = (rxWritePos + 1) % BLE_BUF_SZ;
      }
      //CONSOLE.print(rxValue);
      /*if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++)
          CONSOLE.print(rxValue[i], HEX);
          CONSOLE.print(",");
        }*/
      //CONSOLE.println();
    }
};

void startBLE() {
  // Create the BLE Device
  CONSOLE.println("starting BLE...");
  BLEDevice::init(bleName.c_str());
  BLEDevice::setMTU(BLE_MTU);  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  // https://www.electrosoftcloud.com/en/ble-in-esp32-bluetooth-low-energy-connection/
  #ifdef USE_NIM_BLE
    CONSOLE.println("using NimBLE library");
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                    NIMBLE_PROPERTY::NOTIFY |  // client will be notified if value changed
                    NIMBLE_PROPERTY::READ |  // client can read
                    NIMBLE_PROPERTY::WRITE |  // client can write
                    NIMBLE_PROPERTY::WRITE_NR );  // client can write without our acknowledge (no-response)
  #else
    CONSOLE.println("using ESP32 BLE library");
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                    BLECharacteristic::PROPERTY_NOTIFY |  // client will be notified if value changed
                    BLECharacteristic::PROPERTY_READ |  // client can read
                    BLECharacteristic::PROPERTY_WRITE |  // client can write 
                    BLECharacteristic::PROPERTY_WRITE_NR );  // client can write without our acknowledge (no-response)
    pCharacteristic->addDescriptor(new BLE2902());
  #endif
  pCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);  
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  CONSOLE.println("Waiting a BLE client connection to notify...");
}

#endif 
// ------------------------------- wifi -----------------------------------------------------


void startWIFI() {
  if ((ssid == "") || (pass == "")) return;
  if ((WiFi.status() == WL_CONNECTED) && (WiFi.localIP().toString() != "0.0.0.0")) return;

  if (millis() < nextWifiConnectTime) return;
  nextWifiConnectTime = millis() + 20000;

  for (int i = 0; i < 5; i++) {
    digitalWrite(pinLED, HIGH);
    delay(50);
    digitalWrite(pinLED, LOW);
    delay(50);
  }

  CONSOLE.print("Attempting to connect to WPA SSID: ");
  CONSOLE.println(ssid);

  if (WIFI_STATIC_IP) {
    CONSOLE.println("using static IP");
    if (!WiFi.config(av_local_IP, av_gateway, av_subnet, av_primaryDNS, av_secondaryDNS)) {
      CONSOLE.println("STA Failed to configure");
    }
  } else {
    CONSOLE.println("using dynamic IP");
  }

  WiFi.disconnect(); // disconnect any previous (aborted) connection
  WiFi.begin(ssid.c_str(), pass.c_str());
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    CONSOLE.println("Connection Failed!");
    delay(2000);
    return;
  };

  if (WiFi.status() == WL_CONNECTED) {
    CONSOLE.print("You're connected with SSID=");
    CONSOLE.print(WiFi.SSID());
    CONSOLE.print(" and IP=");
    IPAddress ip = WiFi.localIP();
    CONSOLE.println(ip);
    //server.begin();

    // https://github.com/fhessel/esp32_https_server/issues/11
    //size_t memAvail = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    //HTTPS_LOGE("Available mem: %ld bytes", (long)memAvail);
    CONSOLE.printf("Default Memory:       free size: %8u bytes   largest free block: %8u\n",
      heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
      heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
      // explanation for the following line comes below:
    CONSOLE.printf("Internal 8bit Memory: free size: %8u bytes   largest free block: %8u\n",
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));

    if (server == NULL){ // only start once (do not call for reconnections)
      #ifdef USE_HTTPS
        CONSOLE.println("starting HTTPS server");
        server = new HTTPSServer(&cert, 443, 1);  
      #else
        CONSOLE.println("starting HTTP server");
        server = new HTTPServer(80);        
      #endif
      server->registerNode(nodeRoot);
      server->setDefaultNode(nodeRoot);
      server->start();
    
      ArduinoOTA.setHostname(NAME);
      ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
      })
      .onEnd([]() {
      })
      .onProgress([](unsigned int progress, unsigned int total) {
      })
      .onError([](ota_error_t error) {
      });

      ArduinoOTA.begin();
    }
  }
}


void handleRoot(HTTPRequest * req, HTTPResponse * res) {
  // We will deliver an HTML page
  res->setHeader("Content-Type", "text/html");
  res->setHeader("Access-Control-Allow-Origin", "*");
  byte buffer[256];
  // HTTPReqeust::requestComplete can be used to check whether the
  // body has been parsed completely.
  CONSOLE.print("HTTP rx:");
  String wifiCmd = "";
  while(!(req->requestComplete())) {
    // HTTPRequest::readBytes provides access to the request body.
    // It requires a buffer, the max buffer length and it will return
    // the amount of bytes that have been written to the buffer.
    size_t s = req->readBytes(buffer, 255);
    buffer[s] = '\0';
    CONSOLE.write(buffer, s);
    UART.write(buffer, s);
    wifiCmd += String((char*)buffer);     
  }
  #ifdef USE_MQTT
    mower.tx(wifiCmd);
  #endif
  CONSOLE.println();  
  String cmdResponse;
  unsigned long timeout = millis() + WIFI_TIMEOUT_FIRST_RESPONSE;
  while ( millis() < timeout) {
    if (UART.available()) {
      char ch = UART.read();
      #ifdef USE_MQTT
        mower.rx(ch);
      #endif
      cmdResponse += ch;
      timeout = millis() + WIFI_TIMEOUT_RESPONSE;
    }
    delay(1);
  }
  //simulateArdumowerAnswer(wifiCmd, cmdResponse);  
  CONSOLE.print("UART tx:");
  CONSOLE.println(cmdResponse);
  // Write the response 
  res->print(cmdResponse);
  //res->write(buffer, s);
}


void cmdVersion() {
  String s = F("+VERSION=");
  s += F(VERSION);
  uartSend(s);
}

void cmdTestPacket() {
  String s = F("+TEST");
#ifdef USE_BLE
  bleSend(s);
#endif
}

void cmdName(String aname) {
  bleName = aname;
  String s = F("+NAME=");
  s += bleName;
  uartSend(s);
}

void cmdReset() {
  String s = F("+RESET");
  uartSend(s);
}

void cmdWifi(String mode, String assid, String apass) {
  ssid = assid;
  pass = apass;
  if (mode == "0") {
    String s = F("+WIFI=");
    s += mode + "," + ssid + "," + pass;
    uartSend(s);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    startWIFI();
  } else {
    String s = F("ERROR");
  }
}

void processCmd() {
  cmd.trim();
  while (cmd.length() > 0) {
    if (byte(cmd[0]) > 0) break;
    cmd.remove(0, 1);
  }
  CONSOLE.print(millis());
  CONSOLE.print(" UART rx:");
  CONSOLE.println(cmd);
  /*for (int i=0; i < cmd.length(); i++){
    CONSOLE.print(i);
    CONSOLE.print("=");
    CONSOLE.print(byte(cmd[i]));
    CONSOLE.print("=");
    CONSOLE.println(cmd[i]);
    }*/
  if (cmd.length() < 2) return;
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd.length() < 5) {
    uartSend("OK");
    return;
  }
  int maxCount = 5;
  String params[maxCount];
  int counter = 0;
  int lastIndex = 3;
  for (int i = 0; i < cmd.length(); i++) {
    if (cmd.substring(i, i + 1) == ",") {
      params[counter] = cmd.substring(lastIndex, i);
      lastIndex = i + 1;
      counter++;
    }
    if (i == cmd.length() - 1) {
      params[counter] = cmd.substring(lastIndex, i + 1);
    }
  }
  /*for (int i=0; i < maxCount; i++ ){
    CONSOLE.print("param");
    CONSOLE.print(i);
    CONSOLE.print("=");
    CONSOLE.println(params[i]);
    }*/
  if (params[0].substring(0, 4) == "NAME") cmdName(params[0].substring(4));
  if (params[0] == "VERSION") cmdVersion();
  if (params[0] == "RESET") cmdReset();
  if (params[0] == "TEST") cmdTestPacket();
  if (params[0].substring(0, 4) == "WIFI") cmdWifi(params[0].substring(4), params[1], params[2]);
}


// simulate Ardumower answer (only for BLE testing) 
void simulateArdumowerAnswer(String req, String &resp){
  simPacketCounter++;
  if (req.startsWith("AT+V")){
    resp = "V,Ardumower Sunray,1.0.219,0,78,0x56\n";
  }
  if (req.startsWith("AT+P")){
    resp = "P,0x50\n";  
  }
  if (req.startsWith("AT+M")){        
    resp = "M,0x4d\n";
  }
  if (req.startsWith("AT+S")){        
    if (simPacketCounter % 2 == 0){
      resp = "S,28.60,15.15,-10.24,2.02,2,2,0,0.25,0,15.70,-11.39,0.02,49,-0.05,48,-971195,0x92\n";
    } else {
      resp = "S,27.60,15.15,-10.24,2.02,2,2,0,0.25,0,15.70,-11.39,0.02,49,-0.05,48,-971195,0x91\n";
    }        
  }        
}


void setup() {
  pinMode(pinLED, OUTPUT);
  CONSOLE.begin(115200);       // USB
  UART.begin(115200, SERIAL_8N1, pinGpioRx, pinGpioTx);  // UART

  CONSOLE.println(VERSION);

  CONSOLE.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

#ifdef USE_BLE
  startBLE();
#endif
#ifdef USE_MQTT
   mqtt_setup();
#endif
  //startWIFI();
  relay_setup();
}


void loop() {
#ifdef USE_MQTT
  mower.loop(millis());
#endif
#ifdef USE_BLE
  // -------- BLE -----------------------------
  // disconnecting
  if (!bleConnected && oldBleConnected) {
    CONSOLE.println("waiting for bluetooth stack to get things ready...");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    CONSOLE.println("restart advertising");
    oldBleConnected = false;
  }
  // connecting
  if (bleConnected && !oldBleConnected) {
    // do stuff here on connecting
    oldBleConnected = true;
    CONSOLE.println("BLE connected");
    UART.println(); // clear any remaining characters in Ardumower FIFO 
  }
#endif

  // USB receive
  while (CONSOLE.available()) {
    char ch = CONSOLE.read();
    cmd = cmd + ch;
  }

  // UART receive
  while (UART.available()) {
    char ch = UART.read();
    #ifdef USE_MQTT
      mower.rx(ch);
    #endif    
    if (bleConnected) {
      // BLE client connected
      bleAnswerTimeout = millis() + 100;
      bleAnswer = bleAnswer + ch;
    } else {
      // no BLE client connected
      bleAnswer = "";
      cmd = cmd + ch;
    }
  }

  // LED
  if (!bleConnected) {
    if (millis() > nextLEDTime) {
      nextLEDTime = millis() + 500;
      ledStateNew = !ledStateCurr;
    }
  } else {
    ledStateNew = true;
  }
  if (ledStateCurr != ledStateNew) {
    ledStateCurr = ledStateNew;
    digitalWrite(pinLED, ledStateCurr);
  }


  // BLE->UART bridge
  int num = 0;
  String bleReceivedCmd = "";  
  while (rxReadPos != rxWritePos) {
    char ch = rxBuf[rxReadPos];
    bleReceivedCmd += ch;
    UART.write(ch);
    #ifdef USE_MQTT
      mower.tx(ch);
    #endif    
    rxReadPos = (rxReadPos + 1) % BLE_BUF_SZ;
    num++;
  }
  if (num != 0) {
    CONSOLE.print(millis());
    CONSOLE.print(" BLE rx: ");
    CONSOLE.println(bleReceivedCmd);
  }

// UART->BLE bridge
#ifdef USE_BLE  
  if (bleConnected) {    
    if (bleReceivedCmd != ""){      
      // simulateArdumowerAnswer(bleReceivedCmd, bleAnswer);      
    }
    if (bleAnswer.length() > 0) {
      // BLE client connected
      if ((bleAnswer.endsWith("\n")) || (bleAnswer.endsWith("\r")) || (millis() > bleAnswerTimeout)) {
        if (bleAnswer.length() > 1){
          bleSend(bleAnswer);
        }
        bleAnswer = "";
      }
    }
  }
#endif

  // UART AT-commands
  if (cmd.length() > 0) {
    if (cmd.endsWith("\r")) {
      processCmd();
      cmd = "";
    } else if (cmd.endsWith("\n")) {
      processCmd();
      cmd = "";
    }
  }

  if (millis() > nextPingTime) {
    nextPingTime = millis() + 2000;
    CONSOLE.print(millis());
    CONSOLE.println(" ping");
  }
  
  if (!bleConnected) {
    if (server != NULL) server->loop();
  }

  if (!bleConnected) {
    startWIFI();
  }
  ArduinoOTA.handle();
#ifdef USE_MQTT
  mqtt_loop();
#endif
  relay_loop();
  if (millis() > nextWatchDogResetTime) {
    nextWatchDogResetTime = millis() + 1000;
    esp_task_wdt_reset(); // watch dog reset
  }
}
