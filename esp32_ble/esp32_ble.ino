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
#define VERSION "ESP32 firmware V0.2.1,Bluetooth V4.0 LE"
#define NAME "Ardumower"
#define BLE_MTU 20   // max. transfer bytes per BLE frame

#define BLE_MIN_INTERVAL 1    // connection parameters (tuned for high speed/high power consumption - see: https://support.ambiq.com/hc/en-us/articles/115002907792-Managing-BLE-Connection-Parameters)
#define BLE_MAX_INTERVAL 10
#define BLE_LATENCY      0
#define BLE_TIMEOUT      20  
      
String ssid = "";  // WiFi SSID      (leave empty to not use WiFi)
String pass = "";  // WiFi password  (leave empty to not use WiFi)

#define WIFI_TIMEOUT_FIRST_RESPONSE  200   // fast response times, for more reliable choose: 800     
#define WIFI_TIMEOUT_RESPONSE        50    // fast response times, for more reliable choose: 400

// -----------------------------------------------------------

#define pinGpioRx   16  // UART2
#define pinGpioTx   17  // UART2

//#define pinGpioRx   9   // UART1
//#define pinGpioTx   10  // UART1

//#define pinGpioRx   3   // UART0
//#define pinGpioTx   1   // UART0

#define pinLED   2

#define CONSOLE Serial  // where to send/receive console messages for debugging etc.
#define UART Serial2    // where to send/receive UART data

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>


String cmd;
unsigned long nextInfoTime = 0;
unsigned long nextPingTime = 0;
unsigned long nextLEDTime = 0; 
bool ledStateNew = false;
bool ledStateCurr = false;

// ---- BLE ---------------------------
String bleName = NAME;
BLEServer *pServer = NULL;
BLECharacteristic * pCharacteristic;  

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
WiFiServer server(80);
WiFiClient client;
unsigned long stopClientTime = 0;
  
// ------------------------------- UART -----------------------------------------------------

void uartSend(String s){  
  //if (!bleConnected) return;
  CONSOLE.print(millis());  
  CONSOLE.print(" UART tx:");
  CONSOLE.println(s);
  UART.print(s);
  UART.print("\r\n");  
}

// ------------------------------- BLE -----------------------------------------------------

// send BLE data to BLE client (write data to FIFO)
void bleSend(String s){  
  if (!bleConnected){
    CONSOLE.println("bleSend ignoring: not connected");
    return;
  }
  CONSOLE.print(millis());
  CONSOLE.print(" BLE tx:");
  CONSOLE.println(s);
  for (int i=0; i < s.length(); i++){
    if ( ((txWritePos +1) % BLE_BUF_SZ) == txReadPos){
      CONSOLE.println("BLE: txBuf overflow!");
      break;
    }			
    txBuf[txWritePos] = s[i];                          // push it to the ring buffer  
    txWritePos = (txWritePos + 1) % BLE_BUF_SZ; 			
  }
  bleNotify();
}

// notify BLE client (send next packet from FIFO)
void bleNotify(){
  notifyData = "";
  while ((txReadPos != txWritePos) && (notifyData.length() < BLE_MTU)) {	  
    char ch = txBuf[txReadPos];
    notifyData += ch;
    txReadPos = (txReadPos + 1) % BLE_BUF_SZ;	    
  }
  if (notifyData.length() > 0){
    //CONSOLE.print("notify:");
    //CONSOLE.println(notifyData);
    pCharacteristic->setValue(notifyData.c_str());
    pCharacteristic->notify();
  }  
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param ) {      
      rxReadPos = rxWritePos = 0;
      txReadPos = txWritePos = 0;
      /** After connection we should change the parameters if we (don't) need fast response times.
       *  These settings are 150ms interval, 0 latency, 450ms timout.
       *  Timeout should be a multiple of the interval, minimum is 100ms.
       *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
       *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
       */
      uint16_t connId = pServer->getConnId();
      uint16_t peerMTU = pServer->getPeerMTU(connId);
      // min(1.25ms units),max(1.25ms units),latency(intervals),timeout(10ms units)
      pServer->updateConnParams( param->connect.remote_bda, BLE_MIN_INTERVAL, BLE_MAX_INTERVAL, BLE_LATENCY, BLE_TIMEOUT); // 1, 10, 0, 20 
      CONSOLE.print("---------BLE client connected---------peer mtu=");
      CONSOLE.println(peerMTU);          
      bleConnected = true;        
    };
    void onDisconnect(BLEServer* pServer) {
      bleConnected = false;
      CONSOLE.println("---------BLE client disconnected---------");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {    
    void onStatus(BLECharacteristic* pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code){
      if (s == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY){        
        //CONSOLE.println("onStatus: SUCCESS_NOTIFY");            
        // notify success => send next BLE packet...
        bleNotify();
      } 
    }
    // BLE data received from BLE client => save to FIFO 
    void onWrite(BLECharacteristic *pCharacteristic) {
      //CONSOLE.print("onWrite: ");      
      String rxValue(pCharacteristic->getValue().c_str());      
      for (int i=0; i < rxValue.length(); i++){
			  if ( ((rxWritePos +1) % BLE_BUF_SZ) == rxReadPos){
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

void startBLE(){
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
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, 
        BLECharacteristic::PROPERTY_NOTIFY | 
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR );                          
  pCharacteristic->addDescriptor(new BLE2902());    
  pCharacteristic->setCallbacks(new MyCallbacks());    
  // Start the service    
  pService->start();
  // Start advertising       
  pServer->getAdvertising()->start();        
  CONSOLE.println("Waiting a BLE client connection to notify...");
}

// ------------------------------- wifi -----------------------------------------------------

void startWIFI(){
  if ((ssid == "") || (pass == "")) return;
  if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP().toString() == "0.0.0.0")) {    
    CONSOLE.print("Attempting to connect to WPA SSID: ");
    CONSOLE.println(ssid);

    WiFi.begin(ssid.c_str(), pass.c_str());        
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      CONSOLE.println("Connection Failed!");
      delay(1000);
      return;
    };
    
    if (WiFi.status() == WL_CONNECTED){
      CONSOLE.print("You're connected with SSID=");    
      CONSOLE.print(WiFi.SSID());
      CONSOLE.print(" and IP=");        
      IPAddress ip = WiFi.localIP();    
      CONSOLE.println(ip);   
      //server.listenOnLocalhost();
      server.begin();
     
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

void httpServerStopClient(){
  if (stopClientTime != 0){
    if (millis() < stopClientTime) return;
    CONSOLE.println("stopping HTTP client");
    client.stop();
    stopClientTime = 0;                   
  }  
}

void httpServer(){    
  client = server.available();   // listen for incoming clients
  if (!client) return;

  CONSOLE.println("new HTTP client");           // print a message out the serial port
  String currentLine = "";                // make a String to hold incoming data from the client
  while (client.connected()) {            // loop while the client's connected
    if (client.available()) {             // if there's bytes to read from the client,
      char c = client.read();             // read a byte, then
      //CONSOLE.write(c);                    // print it out the serial monitor
      if (c == '\n') {                    // if the byte is a newline character

        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0) {
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:            
          unsigned long timeout = millis() + 50;        
          String cmd = "";
          while ((client.connected()) && (client.available()) && (millis() < timeout)) {
            char ch = client.read();
            timeout = millis() + 50;
            cmd = cmd + ch;            
          }  
          CONSOLE.print("HTTP rx:");          
          CONSOLE.println(cmd);
          String cmdResponse;
          UART.print(cmd);
          timeout = millis() + WIFI_TIMEOUT_FIRST_RESPONSE; // fast 200 (slow: 800)
          while ( millis() < timeout){
            if (UART.available()){
              char ch = UART.read();
              cmdResponse += ch;
              timeout = millis() + WIFI_TIMEOUT_RESPONSE;  // fast 50 (slow: 400)
            }
            delay(1);
          }
          CONSOLE.print("UART tx:");
          CONSOLE.println(cmdResponse);
          client.print(
            "HTTP/1.1 200 OK\r\n"
            "Access-Control-Allow-Origin: *\r\n"              
            "Content-Type: text/html\r\n"              
            "Connection: close\r\n"  // the connection will be closed after completion of the response
            // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                        
            );
          client.print("Content-length: ");
          client.print(cmdResponse.length());
          client.print("\r\n\r\n");                        
          client.print(cmdResponse);     
          stopClientTime = millis() + 100;                              
          // break out of the while loop:
          break;
        } else {    // if you got a newline, then clear currentLine:
          currentLine = "";
        }
      } else if (c != '\r') {  // if you got anything else but a carriage return character,
        currentLine += c;      // add it to the end of the currentLine
      }        
    }
  }
}


void cmdVersion(){
  String s = F("+VERSION=");
  s += F(VERSION);
  uartSend(s);
}

void cmdTestPacket(){
  String s = F("+TEST");
  bleSend(s);
}

void cmdName(String aname){
  bleName = aname;
  String s = F("+NAME=");  
  s += bleName;
  uartSend(s);
}

void cmdReset(){
  String s = F("+RESET");
  uartSend(s);
}

void cmdWifi(String mode, String assid, String apass){
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

void processCmd(){
  cmd.trim();
  while (cmd.length() > 0) {
    if (byte(cmd[0]) > 0) break;
    cmd.remove(0,1);
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
    if (cmd.substring(i, i+1) == ",") {
      params[counter] = cmd.substring(lastIndex, i);
      lastIndex = i + 1;
      counter++;
     }
    if (i == cmd.length() - 1) {
      params[counter] = cmd.substring(lastIndex, i+1);
    }
  }
  /*for (int i=0; i < maxCount; i++ ){
    CONSOLE.print("param");
    CONSOLE.print(i);
    CONSOLE.print("=");
    CONSOLE.println(params[i]);
  }*/
  if (params[0].substring(0,4) == "NAME") cmdName(params[0].substring(4));  
  if (params[0] == "VERSION") cmdVersion();
  if (params[0] == "RESET") cmdReset();  
  if (params[0] == "TEST") cmdTestPacket();          
  if (params[0].substring(0,4) == "WIFI") cmdWifi(params[0].substring(4),params[1],params[2]);
}



void setup() {  
  pinMode(pinLED, OUTPUT);
  CONSOLE.begin(115200);       // USB
  UART.begin(115200, SERIAL_8N1, pinGpioRx, pinGpioTx);  // UART

  CONSOLE.println(VERSION);

  startBLE();
  //startWIFI();
}


void loop() {     
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
    oldBleConnected = bleConnected;
  }
  // connecting
  if (bleConnected && !oldBleConnected) {
    // do stuff here on connecting
    bleConnected = bleConnected;
  }

  // USB receive
  while (CONSOLE.available()){
    char ch = CONSOLE.read();          
    cmd = cmd + ch;
  }

  // UART receive   
  while (UART.available()){    
    char ch = UART.read();          
    if (bleConnected){
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
  if (bleConnected){
    if (bleAnswer.length() > 0){
      // BLE client connected
      if ((bleAnswer.endsWith("\n")) || (bleAnswer.endsWith("\r")) || (millis() > bleAnswerTimeout) || (bleAnswer.length() >= BLE_MTU)) {
        bleSend(bleAnswer);
        bleAnswer = "";
      }
    }
  }

  // LED
  if (!bleConnected){
    if (millis() > nextLEDTime){
      nextLEDTime = millis() + 500;
      ledStateNew = !ledStateCurr;      
    }
  } else {    
    ledStateNew = true;    
  }
  if (ledStateCurr != ledStateNew){
    ledStateCurr = ledStateNew;
    digitalWrite(pinLED, ledStateCurr);
  }


  // BLE->UART bridge
  int num = 0;
  String s = "";
  while (rxReadPos != rxWritePos){	  
    char ch = rxBuf[rxReadPos];
    s += ch;
    UART.write(ch);
    rxReadPos = (rxReadPos + 1) % BLE_BUF_SZ;	
    num++;
  }
  if (num != 0){
    CONSOLE.print(millis());  
    CONSOLE.print(" BLE rx: ");
    CONSOLE.println(s);
  }
  
  // UART AT-commands
  if (cmd.length()>0){        
    if (cmd.endsWith("\r")){
      processCmd();      
      cmd = "";
    } else if (cmd.endsWith("\n")){
      processCmd();
      cmd = "";
    } 
  }
  
  if (millis() > nextPingTime){
    nextPingTime = millis() + 2000;
    CONSOLE.print(millis());
    CONSOLE.println(" ping");        
  }

  httpServerStopClient();
  if (!bleConnected) httpServer();

  startWIFI();
  ArduinoOTA.handle();

}


