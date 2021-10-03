/*   ESP32 firmware providing:
     1. BLE UART bridge (GATT server UART)
     2. WiFi UART bridge (HTTP server UART)


  Steps to install ESP32 for Arduino IDE:
     1. Arduino IDE: File->Preferences:  Add to board manager URLs: ",https://dl.espressif.com/dl/package_esp32_index.json"
     2. Choose "Tools->Board->Boards Manager"
     3. Add board "esp32" (IMPORTANT!!! choose v1.0.4, latest v1.0.6 does not seem to connect to WiFi access point! )
     4. Choose Board "ESP32 Dev Module"  (if upload does not work: PRESS EN+BOOT, release EN  on your ESP32)
     5. Choose Partition Scheme "Minimal SPIFFS"  (otherwise you may get 'memory space errors' in the Arduino IDE)
    (also see: https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md )
     6. Choose Port (Windows NOTE: if the port is not shown you may have to install drivers: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
     7. Choose "Tools->Manager Libraries..."
     8. Add library "ESP32_HTTPS_Server"
     9. Add library "NimBLE-Arduino"

  wiring (also see wiring image in Github folder):
  ESP32 Rx2 (GPIO16) ---  Ardumower PCB1.3 Bluetooth conn RX   (3.3v level)
  ESP32 Tx2 (GPIO17) ---  Ardumower PCB1.3 Bluetooth conn TX   (3.3v level)
  ESP32 GND          ---  Ardumower PCB1.3 Bluetooth conn GND
  ESP32 5V  (Vin)    ---  Ardumower PCB1.3 Bluetooth conn 5V


  Note: ESP32 D1 Mini (china cheapo) can work as a comm/BLE chip.
  Data is reported and mower can be steered using app.
  Things to remember are:
  - need to wait few secs after mower reboots before connecting (CRC issues)
  - need to close/reopen app if the phone disconnects (CRC issues)


  serial protocol:

  ---COMMANDS---                                                ---ANSWER---
                                AT\r\n                          OK\r\n
  request version               AT+VERSION\r\n                  +VERSION=ESP32 firmware V0.1,Bluetooth V4.0 LE\r\n
  change BLE name               AT+NAMEname\r\n                 +NAME=name\r\n
  reset module                  AT+RESET\r\n                    +RESET\r\n
  send BLE test packet          AT+TEST\r\n                     +TEST\r\n
  connect to wifi               AT+WIFImode,ssid,pass\r\n       +WIFI=mode,ssid,pass\r\n
*/

// ---------- configuration ----------------------------------
#define VERSION "ESP32 firmware V0.3.0,Bluetooth V4.0 LE"
#define NAME "Ardumower"


#define USE_BLE 1    // comment this line to remove BLE support
//#define USE_NIM_BLE   // use NimBLE library instead of ESP32 library? 
#define BLE_MTU 20   // max. transfer bytes per BLE frame

#define BLE_MIN_INTERVAL 2    // connection parameters (tuned for high speed/high power consumption - see: https://support.ambiq.com/hc/en-us/articles/115002907792-Managing-BLE-Connection-Parameters)
#define BLE_MAX_INTERVAL 10
#define BLE_LATENCY      0
#define BLE_TIMEOUT      30


//IP WiFi:
//#define WIFI_STATIC_IP true  // activate this for static IP
#define WIFI_STATIC_IP false // activate this for dynamic IP

// configure below IPs if using static IP
IPAddress av_local_IP(10, 0, 100, 11);
IPAddress av_gateway(10, 0, 100, 1);
IPAddress av_subnet(255, 255, 255, 0);
IPAddress av_primaryDNS(8, 8, 8, 8); //optional
IPAddress av_secondaryDNS(8, 8, 4, 4); //optional

String ssid = "yourSSID";  // WiFi SSID      (leave empty ("") to not use WiFi)
String pass = "yourPASSWORD";  // WiFi password  (leave empty ("") to not use WiFi)

#define WIFI_TIMEOUT_FIRST_RESPONSE  800   // fast response times (500), for more reliable choose: 800     
#define WIFI_TIMEOUT_RESPONSE        400    // fast response times (100), for more reliable choose: 400

// #define USE_HTTPS  // comment this line to use HTTP

// -----------------------------------------------------------

#define pinGpioRx   16    // UART2 / GPIO16 / IO16
#define pinGpioTx   17    // UART2 / GPIO17 / IO17

//#define pinGpioRx   9   // UART1 / GPIO9  / SD2
//#define pinGpioTx   10  // UART1 / GPIO10 / SD3

//#define pinGpioRx   3   // UART0 / GPIO3  / RXD0
//#define pinGpioTx   1   // UART0 / GPIO1  / TXD0

#define pinLED   2

#define CONSOLE Serial  // where to send/receive console messages for debugging etc.
#define UART Serial2    // where to send/receive UART data

// watch dog timeout (WDT) in seconds
#define WDT_TIMEOUT 60

// Include certificate data 
#include "cert.h"
#include "private_key.h"

#include <WiFi.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>

#ifdef USE_HTTPS
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

String cmd;
unsigned long nextInfoTime = 0;
unsigned long nextPingTime = 0;
unsigned long nextLEDTime = 0;
unsigned long nextWatchDogResetTime = 0;
bool ledStateNew = false;
bool ledStateCurr = false;

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

int rxReadPos = 0;
int rxWritePos = 0;
byte rxBuf[BLE_BUF_SZ];

int txReadPos = 0;
int txWritePos = 0;
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
WiFiClient client;
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
  notifyData = "";
  while ((txReadPos != txWritePos) && (notifyData.length() < BLE_MTU)) {
    char ch = txBuf[txReadPos];
    notifyData += ch;
    txReadPos = (txReadPos + 1) % BLE_BUF_SZ;
  }
  if (notifyData.length() > 0) {
    //CONSOLE.print("notify:");
    //CONSOLE.println(notifyData);
    pCharacteristic->setValue(notifyData.c_str());
    pCharacteristic->notify();
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    #ifdef USE_NIM_BLE
      void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    #else
      void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param ) {
    #endif
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
      #else
        uint16_t connId = pServer->getConnId();
        uint16_t peerMTU = pServer->getPeerMTU(connId);
        pServer->updateConnParams( param->connect.remote_bda, BLE_MIN_INTERVAL, BLE_MAX_INTERVAL, BLE_LATENCY, BLE_TIMEOUT); // 1, 10, 0, 20    
      #endif      
      CONSOLE.println("---------BLE client connected---------");
      //CONSOLE.println(peerMTU);
      bleConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      bleConnected = false;
      CONSOLE.println("---------BLE client disconnected---------");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onStatus(BLECharacteristic* pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code) {
      if (s == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY) {
        //CONSOLE.println("onStatus: SUCCESS_NOTIFY");
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
  //BLEDevice::setMTU(517);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  #ifdef USE_NIM_BLE
    CONSOLE.println("using NimBLE library");
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                    NIMBLE_PROPERTY::NOTIFY |
                    NIMBLE_PROPERTY::READ |
                    NIMBLE_PROPERTY::WRITE |
                    NIMBLE_PROPERTY::WRITE_NR );    
  #else
    CONSOLE.println("using ESP32 BLE library");
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_WRITE |
                    BLECharacteristic::PROPERTY_WRITE_NR );
    pCharacteristic->addDescriptor(new BLE2902());
  #endif
  pCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  CONSOLE.println("Waiting a BLE client connection to notify...");
}

#endif 
// ------------------------------- wifi -----------------------------------------------------


void startWIFI() {
  if ((ssid == "") || (pass == "")) return;
  if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP().toString() == "0.0.0.0")) {

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

      #ifdef USE_HTTPS
        CONSOLE.println("using HTTPS");
        server = new HTTPSServer(&cert, 443, 1);  
      #else
        CONSOLE.println("using HTTP");
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
  while(!(req->requestComplete())) {
    // HTTPRequest::readBytes provides access to the request body.
    // It requires a buffer, the max buffer length and it will return
    // the amount of bytes that have been written to the buffer.
    size_t s = req->readBytes(buffer, 256);
    CONSOLE.write(buffer, s);
    UART.write(buffer, s);     
  }
  CONSOLE.println();  
  String cmdResponse;
  UART.print(cmd);
  unsigned long timeout = millis() + WIFI_TIMEOUT_FIRST_RESPONSE;
  while ( millis() < timeout) {
    if (UART.available()) {
      char ch = UART.read();
      cmdResponse += ch;
      timeout = millis() + WIFI_TIMEOUT_RESPONSE;
    }
    delay(1);
  }
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
  //startWIFI();
}


void loop() {
#ifdef USE_BLE
  // -------- BLE -----------------------------
  // disconnecting
  if (!bleConnected && oldBleConnected) {
    // advertising
    //delay(500); // give the bluetooth stack the chance to get things ready
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    CONSOLE.println("start advertising");
    oldBleConnected = false;
  }
  // connecting
  if (bleConnected && !oldBleConnected) {
    // do stuff here on connecting
    oldBleConnected = true;
    CONSOLE.println("BLE connected");
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

  // UART->BLE bridge
#ifdef USE_BLE
  if (bleConnected) {
    if (bleAnswer.length() > 0) {
      // BLE client connected
      if ((bleAnswer.endsWith("\n")) || (bleAnswer.endsWith("\r")) || (millis() > bleAnswerTimeout) || (bleAnswer.length() >= BLE_MTU)) {
        bleSend(bleAnswer);
        bleAnswer = "";
      }
    }
  }
#endif

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
  String s = "";
  while (rxReadPos != rxWritePos) {
    char ch = rxBuf[rxReadPos];
    s += ch;
    UART.write(ch);
    rxReadPos = (rxReadPos + 1) % BLE_BUF_SZ;
    num++;
  }
  if (num != 0) {
    CONSOLE.print(millis());
    CONSOLE.print(" BLE rx: ");
    CONSOLE.println(s);
  }

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

  startWIFI();
  ArduinoOTA.handle();

  if (millis() > nextWatchDogResetTime) {
    nextWatchDogResetTime = millis() + 1000;
    esp_task_wdt_reset(); // watch dog reset
  }
}
