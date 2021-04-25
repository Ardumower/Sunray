/*   ESP32 BLE-UART bridge firmware (GATT server UART)
     
Steps to install ESP32 for Arduino IDE:
     1. Arduino IDE: File->Preferences:  Add to board manager URLs: ",https://dl.espressif.com/dl/package_esp32_index.json"
     2. Choose "Tools->Board->Boards Manager"
     3. Add board "esp32"
     4. Choose Board "ESP32 Dev Module"
        https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md

    Arduino IDE: choose ESP32 Dev Module  (if upload does not work: PRESS EN+BOOT, release EN)

wiring: 
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

---COMMANDS---                                      ---ANSWER---
                              AT\r\n                OK\r\n
request version               AT+VERSION\r\n        +VERSION=ESP32 firmware V0.1,Bluetooth V4.0 LE\r\n
change BLE name               AT+Nname\r\n          +NAME=name\r\n
reset module                  AT+RESET\r\n          +RESET\r\n      
send BLE test packet          AT+TEST\r\n           +TEST\r\n      
*/

#include <EEPROM.h>


#define VERSION "ESP32 firmware V0.1,Bluetooth V4.0 LE"
#define NAME "Ardumower"
#define MTU 20   // max. transfer bytes per BLE frame

#define pinGpioRx   16  // UART2
#define pinGpioTx   17  // UART2

//#define pinGpioRx   9   // UART1
//#define pinGpioTx   10  // UART1

//#define pinGpioRx   3   // UART0
//#define pinGpioTx   1   // UART0

#ifdef USE_WIFI
  #include <WiFi.h>
#endif

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

#ifdef USE_WIFI
  const char* ssid     = "ssid";
  const char* password = "password";
  WiFiServer server(80);
#endif  

String cmd;
unsigned long nextInfoTime = 0;
unsigned long nextPingTime = 0;

// ---- BLE ---------------------------
BLEServer *pServer = NULL;
BLECharacteristic * pCharacteristic;  

String bleAnswer = "";
unsigned long bleAnswerTimeout = 0;
bool deviceConnected = false;
bool oldDeviceConnected = false;
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


// ------ EEPROM --------------------------
bool loadEEPROM(){
  Serial.println(F("loadEEPROM"));
  int addr = 0;
  unsigned short id = 0;
  id = EEPROM.readUShort(addr);  
  if (id != 0xAAE2) {
    Serial.println(F("no valid EEPROM data found"));
    return false;
  }
  Serial.println(F("valid EEPROM data found"));
  addr += sizeof(id);
  /*totalAh = EEPROM.readFloat(addr);
  addr += sizeof(totalAh);
  AhAccumulated = EEPROM.readFloat(addr);
  addr += sizeof(AhAccumulated);
  
  Serial.print(F("totalAh="));
  Serial.println(totalAh);      
  Serial.print(F("AhAccumulated="));
  Serial.println(AhAccumulated);*/      
  return true;
}


void saveEEPROM(){
  Serial.println(F("saveEEPROM"));
  float floatValue;
  int addr = 0;
  unsigned short id = 0xAAE2;
  if (EEPROM.readUShort(addr) != id) EEPROM.writeUShort(addr, id);
  addr += sizeof(id);
  /*floatValue = EEPROM.readFloat(addr);
  if (abs(totalAh-floatValue)>0.5) EEPROM.writeFloat(addr, totalAh);        
  addr += sizeof(totalAh);
  floatValue = EEPROM.readFloat(addr);
  if (abs(AhAccumulated-floatValue)>0.5) EEPROM.writeFloat(addr, AhAccumulated);      
  addr += sizeof(AhAccumulated);*/
  EEPROM.commit();
}


void uartSend(String s){  
  //if (!deviceConnected) return;
  Serial.print(millis());  
  Serial.print(" UART tx:");
  Serial.println(s);
  Serial2.print(s);
  Serial2.print("\r\n");  
}

// send BLE data to BLE client (write data to FIFO)
void bleSend(String s){  
  if (!deviceConnected){
    Serial.println("bleSend ignoring: not connected");
    return;
  }
  Serial.print(millis());
  Serial.print(" BLE tx:");
  Serial.println(s);
  for (int i=0; i < s.length(); i++){
    if ( ((txWritePos +1) % BLE_BUF_SZ) == txReadPos){
      Serial.println("BLE: txBuf overflow!");
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
  while ((txReadPos != txWritePos) && (notifyData.length() < MTU)) {	  
    char ch = txBuf[txReadPos];
    notifyData += ch;
    txReadPos = (txReadPos + 1) % BLE_BUF_SZ;	    
  }
  if (notifyData.length() > 0){
    //Serial.print("notify:");
    //Serial.println(notifyData);
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
      pServer->updateConnParams( param->connect.remote_bda, 1, 10, 0, 20); 
      Serial.print("---------BLE client connected---------peer mtu=");
      Serial.println(peerMTU);          
      deviceConnected = true;        
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("---------BLE client disconnected---------");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {    
    void onStatus(BLECharacteristic* pCharacteristic, BLECharacteristicCallbacks::Status s, uint32_t code){
      if (s == BLECharacteristicCallbacks::Status::SUCCESS_NOTIFY){        
        //Serial.println("onStatus: SUCCESS_NOTIFY");            
        // notify success => send next BLE packet...
        bleNotify();
      } 
    }
    // BLE data received from BLE client => save to FIFO 
    void onWrite(BLECharacteristic *pCharacteristic) {
      //Serial.print("onWrite: ");      
      String rxValue(pCharacteristic->getValue().c_str());      
      for (int i=0; i < rxValue.length(); i++){
			  if ( ((rxWritePos +1) % BLE_BUF_SZ) == rxReadPos){
			    Serial.println("BLE: rxBuf overflow!");
			    break;
			  }			
			  rxBuf[rxWritePos] = rxValue[i];                          // push it to the ring buffer  
			  rxWritePos = (rxWritePos + 1) % BLE_BUF_SZ; 			
		  }
      //Serial.print(rxValue);
      /*if (rxValue.length() > 0) {        
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i], HEX);                  
          Serial.print(",");
      }*/      
      //Serial.println();      
    }
};


void cmdClearEEPROM(){
  Serial.print(F("clearEEPROM"));  
  unsigned long nextTime = 0;
  for (int addr=0; addr < 4096; addr++) {
    if (millis() > nextTime){
      nextTime = millis() + 2000;          
      Serial.print(F(".")); 
    }    
    EEPROM.writeByte(addr, 0);      
  }
  Serial.println(F("ok")); 
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

void cmdName(){
  String s = F("+NAME=");
  uartSend(s);
}

void cmdReset(){
  String s = F("+RESET");
  uartSend(s);
}

void processCmd(){
  cmd.trim();
  while (cmd.length() > 0) {
    if (byte(cmd[0]) > 0) break;
    cmd.remove(0,1);
  }
  Serial.print(millis());
  Serial.print(" UART rx:");
  Serial.println(cmd);
  /*for (int i=0; i < cmd.length(); i++){
    Serial.print(i);
    Serial.print("=");
    Serial.print(byte(cmd[i]));
    Serial.print("=");    
    Serial.println(cmd[i]);
  }*/
  if (cmd.length() < 2) return;  
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd.length() < 5) {
    uartSend("OK");
    return;
  } 
  if (cmd[2] != '+') return;    
  if (cmd[3] == 'N') cmdName();  
  if (cmd[3] == 'V') cmdVersion();
  if (cmd[3] == 'R') cmdReset();  
  if (cmd[3] == 'T') cmdTestPacket();    
  //if (cmd[3] == 'Y') cmdClearEEPROM();    
}



void setup() {  
  Serial.begin(115200);       // USB
  Serial2.begin(115200, SERIAL_8N1, pinGpioRx, pinGpioTx);  // UART

  Serial.println(VERSION);
  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  //if (!loadEEPROM()){
  //  cmdClearEEPROM();
  //}  


#ifdef USE_WIFI
    // ---- WIFI ------------------------
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());    
    server.begin();     
#endif    

    // ---- BLE ------------------
    // Create the BLE Device
    Serial.println("starting BLE...");
    BLEDevice::init(NAME);
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
    Serial.println("Waiting a BLE client connection to notify...");
}


void loop() {     
  // -------- BLE -----------------------------  
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    //delay(500); // give the bluetooth stack the chance to get things ready
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising(); 
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  // USB receive
  while (Serial.available()){
    char ch = Serial.read();          
    cmd = cmd + ch;
  }

  // UART receive  
  while (Serial2.available()){    
    char ch = Serial2.read();          
    if (deviceConnected){
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
  if (deviceConnected){
    if (bleAnswer.length() > 0){
      // BLE client connected
      if ((bleAnswer.endsWith("\n")) || (bleAnswer.endsWith("\r")) || (millis() > bleAnswerTimeout) || (bleAnswer.length() >= MTU)) {
        bleSend(bleAnswer);
        bleAnswer = "";
      }
    }
  }

  // BLE->UART bridge
  int num = 0;
  String s = "";
  while (rxReadPos != rxWritePos){	  
    char ch = rxBuf[rxReadPos];
    s += ch;
    Serial2.write(ch);
    rxReadPos = (rxReadPos + 1) % BLE_BUF_SZ;	
    num++;
  }
  if (num != 0){
    Serial.print(millis());  
    Serial.print(" BLE rx: ");
    Serial.println(s);
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
  
  /*if (millis() > nextEEPROMTime){        
      nextEEPROMTime = millis() + 60000 * 10; // 10 minutes
      saveEEPROM();    
  } */    

  if (millis() > nextPingTime){
    nextPingTime = millis() + 2000;
    Serial.print(millis());
    Serial.println(" ping");        
  }
}

