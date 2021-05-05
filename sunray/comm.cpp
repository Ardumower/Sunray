#include "comm.h"
#include "config.h"
#include "robot.h"
#include "reset.h"
#ifdef __linux__
  #include <BridgeClient.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "RingBuffer.h"

unsigned long nextInfoTime = 0;
bool triggerWatchdog = false;

int encryptMode = 0; // 0=off, 1=encrypt
int encryptPass = PASS; 
int encryptChallenge = 0;
int encryptKey = 0;

bool simFaultyConn = false; // simulate a faulty connection?
int simFaultConnCounter = 0;

String cmd;
String cmdResponse;

// use a ring buffer to increase speed and reduce memory allocation
ERingBuffer buf(8);
int reqCount = 0;                // number of requests received
unsigned long stopClientTime = 0;
float statControlCycleTime = 0; 
float statMaxControlCycleTime = 0; 

// mqtt
#define MSG_BUFFER_SIZE	(50)
char mqttMsg[MSG_BUFFER_SIZE];
unsigned long nextPublishTime = 0;

// wifi client
WiFiEspClient wifiClient;
unsigned long nextWifiClientCheckTime = 0;


// answer Bluetooth with CRC
void cmdAnswer(String s){  
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);  
  s += F("\r\n");             
  //CONSOLE.print(s);  
  cmdResponse = s;
}


// request operation
void cmdControl(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int mow=-1;          
  int op = -1;
  float wayPerc = -1;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          if (intValue >= 0) {
            motor.enableMowMotor = (intValue == 1);
            motor.setMowState( (intValue == 1) );
          }
      } else if (counter == 2){                                      
          if (intValue >= 0) op = intValue; 
      } else if (counter == 3){                                      
          if (floatValue >= 0) setSpeed = floatValue; 
      } else if (counter == 4){                                      
          if (intValue >= 0) fixTimeout = intValue; 
      } else if (counter == 5){
          if (intValue >= 0) finishAndRestart = (intValue == 1);
      } else if (counter == 6){
          if (floatValue >= 0) maps.setMowingPointPercent(floatValue);
      } else if (counter == 7){
          if (intValue > 0) maps.skipNextMowingPoint();
      } else if (counter == 8){
          if (intValue >= 0) sonar.enabled = (intValue == 1);
      }
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  CONSOLE.println(angular);*/    
  if (op >= 0) setOperation((OperationType)op, false, true);
  String s = F("C");
  cmdAnswer(s);
}

// request motor 
void cmdMotor(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  float linear=0;
  float angular=0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      float value = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          linear = value;
      } else if (counter == 2){
          angular = value;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("linear=");
  CONSOLE.print(linear);
  CONSOLE.print(" angular=");
  CONSOLE.println(angular);*/
  motor.setLinearAngularSpeed(linear, angular, false);
  String s = F("M");
  cmdAnswer(s);
}

void cmdMotorTest(){
  String s = F("E");
  cmdAnswer(s);
  motor.test();  
}

void cmdMotorPlot(){
  String s = F("Q");
  cmdAnswer(s);
  motor.plot();  
}

void cmdSensorTest(){
  String s = F("F");
  cmdAnswer(s);
  sensorTest();  
}


// request waypoint
void cmdWaypoint(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int widx=0;  
  float x=0;
  float y=0;
  bool success = true;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){            
      float intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          widx = intValue;
      } else if (counter == 2){
          x = floatValue;
      } else if (counter == 3){
          y = floatValue;
          if (!maps.setPoint(widx, x, y)){
            success = false;
            break;
          }          
          widx++;
          counter = 1;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }      
  /*CONSOLE.print("waypoint (");
  CONSOLE.print(widx);
  CONSOLE.print("/");
  CONSOLE.print(count);
  CONSOLE.print(") ");
  CONSOLE.print(x);
  CONSOLE.print(",");
  CONSOLE.println(y);*/  
  
  String s = F("W,");
  s += widx;              
  cmdAnswer(s);       
  
  if (!success){   
    stateSensor = SENS_MEM_OVERFLOW;
    setOperation(OP_ERROR);
  } 
}


// request waypoints count
void cmdWayCount(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){            
      float intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();      
      if (counter == 1){                            
          if (!maps.setWayCount(WAY_PERIMETER, intValue)) return;                
      } else if (counter == 2){
          if (!maps.setWayCount(WAY_EXCLUSION, intValue)) return;                
      } else if (counter == 3){
          if (!maps.setWayCount(WAY_DOCK, intValue)) return;                
      } else if (counter == 4){
          if (!maps.setWayCount(WAY_MOW, intValue)) return;                
      } else if (counter == 5){
          if (!maps.setWayCount(WAY_FREE, intValue)) return;                
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }        
  String s = F("N");    
  cmdAnswer(s);         
  //maps.dump();
}


// request exclusion count
void cmdExclusionCount(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int widx=0;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){            
      float intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      float floatValue = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
          widx = intValue;
      } else if (counter == 2){
          if (!maps.setExclusionLength(widx, intValue)) return;          
          widx++;
          counter = 1;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }        
  String s = F("X,");
  s += widx;              
  cmdAnswer(s);         
}


// request position mode
void cmdPosMode(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, idx+1).toInt();
      double doubleValue = cmd.substring(lastCommaIdx+1, idx+1).toDouble();
      if (counter == 1){                            
          absolutePosSource = bool(intValue);
      } else if (counter == 2){                                      
          absolutePosSourceLon = doubleValue; 
      } else if (counter == 3){                                      
          absolutePosSourceLat = doubleValue; 
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }        
  CONSOLE.print("absolutePosSource=");
  CONSOLE.print(absolutePosSource);
  CONSOLE.print(" lon=");
  CONSOLE.print(absolutePosSourceLon, 8);
  CONSOLE.print(" lat=");
  CONSOLE.println(absolutePosSourceLat, 8);
  String s = F("P");
  cmdAnswer(s);
}

// request version
void cmdVersion(){  
#ifdef ENABLE_PASS
  if (encryptMode == 0){
    encryptMode = 1;
    randomSeed(millis());
    while (true){
      encryptChallenge = random(0, 200); // random number between 0..199
      encryptKey = encryptPass % encryptChallenge;   
      if ( (encryptKey >= 1) && (encryptKey <= 94) ) break;  // random key between 1..94
    }
  }
#endif
  String s = F("V,");
  s += F(VER);
  s += F(",");  
  s += encryptMode;
  s += F(",");
  s += encryptChallenge;
  CONSOLE.print("sending encryptMode=");
  CONSOLE.print(encryptMode);
  CONSOLE.print(" encryptChallenge=");  
  CONSOLE.println(encryptChallenge);
  cmdAnswer(s);
}

// request add obstacle
void cmdObstacle(){
  String s = F("O");
  cmdAnswer(s);  
  triggerObstacle();  
}

// perform pathfinder stress test
void cmdStressTest(){
  String s = F("Z");
  cmdAnswer(s);  
  maps.stressTest();  
}

// perform hang test (watchdog should trigger)
void cmdTriggerWatchdog(){
  String s = F("Y");
  cmdAnswer(s);  
  setOperation(OP_IDLE);
  triggerWatchdog = true;
}

// perform hang test (watchdog should trigger)
void cmdGNSSReboot(){
  String s = F("Y2");
  cmdAnswer(s);  
  CONSOLE.println("GNNS reboot");
  gps.reboot();
}

// switch-off robot
void cmdSwitchOffRobot(){
  String s = F("Y3");
  cmdAnswer(s);  
  setOperation(OP_IDLE);
  battery.switchOff();
}

// kidnap test (kidnap detection should trigger)
void cmdKidnap(){
  String s = F("K");
  cmdAnswer(s);  
  CONSOLE.println("kidnapping robot - kidnap detection should trigger");
  stateX = 0;  
  stateY = 0;
}

// toggle GPS solution (invalid,float,fix) for testing
void cmdToggleGPSSolution(){
  String s = F("G");
  cmdAnswer(s);  
  CONSOLE.println("toggle GPS solution");
  switch (gps.solution){
    case UBLOX::SOL_INVALID:  
      gps.solutionAvail = true;
      gps.solution = UBLOX::SOL_FLOAT;
      gps.relPosN = stateY - 2.0;  // simulate pos. solution jump
      gps.relPosE = stateX - 2.0;
      lastFixTime = millis();
      stateGroundSpeed = 0.1;
      break;
    case UBLOX::SOL_FLOAT:  
      gps.solutionAvail = true;
      gps.solution = UBLOX::SOL_FIXED;
      stateGroundSpeed = 0.1;
      gps.relPosN = stateY + 2.0;  // simulate undo pos. solution jump
      gps.relPosE = stateX + 2.0;
      break;
    case UBLOX::SOL_FIXED:  
      gps.solutionAvail = true;
      gps.solution = UBLOX::SOL_INVALID;
      break;
  }
}


// request summary
void cmdSummary(){
  String s = F("S,");
  s += battery.batteryVoltage;  
  s += ",";
  s += stateX;
  s += ",";
  s += stateY;
  s += ",";
  s += stateDelta;
  s += ",";
  s += gps.solution;
  s += ",";
  s += stateOp;
  s += ",";
  s += maps.mowPointsIdx;
  s += ",";
  s += (millis() - gps.dgpsAge)/1000.0;
  s += ",";
  s += stateSensor;
  s += ",";
  s += maps.targetPoint.x();
  s += ",";
  s += maps.targetPoint.y();  
  s += ",";
  s += gps.accuracy;  
  s += ",";
  s += gps.numSV;  
  s += ",";
  if (stateOp == OP_CHARGE) {
    s += "-";
    s += battery.chargingCurrent;
  } else {
    s += motor.motorsSenseLP;
  }
  s += ",";
  s += gps.numSVdgps;  
  s += ",";
  s += maps.mapCRC;
  cmdAnswer(s);  
}

// request statistics
void cmdStats(){
  String s = F("T,");
  s += statIdleDuration;  
  s += ",";
  s += statChargeDuration;
  s += ",";
  s += statMowDuration;
  s += ",";
  s += statMowDurationFloat;
  s += ",";
  s += statMowDurationFix;
  s += ",";
  s += statMowFloatToFixRecoveries;
  s += ",";  
  s += statMowDistanceTraveled;  
  s += ",";  
  s += statMowMaxDgpsAge;
  s += ",";
  s += statImuRecoveries;
  s += ",";
  s += statTempMin;
  s += ",";
  s += statTempMax;
  s += ",";
  s += gps.chksumErrorCounter;
  s += ",";
  //s += ((float)gps.dgpsChecksumErrorCounter) / ((float)(gps.dgpsPacketCounter));
  s += gps.dgpsChecksumErrorCounter;
  s += ",";
  s += statMaxControlCycleTime;
  s += ",";
  s += SERIAL_BUFFER_SIZE;
  s += ",";
  s += statMowDurationInvalid;
  s += ",";
  s += statMowInvalidRecoveries;
  s += ",";
  s += statMowObstacles;
  s += ",";
  s += freeMemory();
  s += ",";
  s += getResetCause();
  s += ",";
  s += statGPSJumps;
  s += ",";
  s += statMowSonarCounter;
  s += ",";
  s += statMowBumperCounter;
  s += ",";
  s += statMowGPSMotionTimeoutCounter;
  cmdAnswer(s);  
}

// clear statistics
void cmdClearStats(){
  String s = F("L");
  statIdleDuration = 0;
  statChargeDuration = 0;
  statMowDuration = 0;
  statMowDurationInvalid = 0;
  statMowDurationFloat = 0;
  statMowDurationFix = 0;
  statMowFloatToFixRecoveries = 0;
  statMowInvalidRecoveries = 0;
  statMowDistanceTraveled = 0;
  statMowMaxDgpsAge = 0;
  statImuRecoveries = 0;
  statTempMin = 9999;
  statTempMax = -9999;
  gps.chksumErrorCounter = 0;
  gps.dgpsChecksumErrorCounter = 0;
  statMaxControlCycleTime = 0;
  statMowObstacles = 0;
  statMowBumperCounter = 0; 
  statMowSonarCounter = 0;
  statMowLiftCounter = 0;
  statMowGPSMotionTimeoutCounter = 0;
  statGPSJumps = 0;
  cmdAnswer(s);  
}


// process request
void processCmd(bool checkCrc, bool decrypt){
  cmdResponse = "";      
  if (cmd.length() < 4) return;
#ifdef ENABLE_PASS
  if (decrypt){
    String s = cmd.substring(0,4);
    if ( s != "AT+V"){
      if (encryptMode == 1){
        // decrypt        
        for (int i=0; i < cmd.length(); i++) {
          if ( (byte(cmd[i]) >= 32) && (byte(cmd[i]) <= 126) ){  // ASCII between 32..126
            int code = byte(cmd[i]);
            code -= encryptKey;
            if (code <= 31) code = 126 + (code-31);
            cmd[i] = char(code);  
          }
        }
        CONSOLE.print("decrypt:");
        CONSOLE.println(cmd);
      }
    } 
  }
#endif
  byte expectedCrc = 0;
  int idx = cmd.lastIndexOf(',');
  if (idx < 1){
    if (checkCrc){
      CONSOLE.println("CRC ERROR");
      return;
    }
  } else {
    for (int i=0; i < idx; i++) expectedCrc += cmd[i];  
    String s = cmd.substring(idx+1, idx+5);
    int crc = strtol(s.c_str(), NULL, 16);
    bool crcErr = false;
    simFaultConnCounter++;
    if ((simFaultyConn) && (simFaultConnCounter % 10 == 0)) crcErr = true;
    if ((expectedCrc != crc) && (checkCrc)) crcErr = true;      
    if (crcErr) {
      CONSOLE.print("CRC ERROR");
      CONSOLE.print(crc,HEX);
      CONSOLE.print(",");
      CONSOLE.print(expectedCrc,HEX);
      CONSOLE.println();
      return;        
    } else {
      // remove CRC
      cmd = cmd.substring(0, idx);
      //CONSOLE.println(cmd);
    }    
  }     
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd[2] != '+') return;
  if (cmd[3] == 'S') cmdSummary();
  if (cmd[3] == 'M') cmdMotor();
  if (cmd[3] == 'C') cmdControl();
  if (cmd[3] == 'W') cmdWaypoint();
  if (cmd[3] == 'N') cmdWayCount();
  if (cmd[3] == 'X') cmdExclusionCount();
  if (cmd[3] == 'V') cmdVersion();  
  if (cmd[3] == 'P') cmdPosMode();  
  if (cmd[3] == 'T') cmdStats();
  if (cmd[3] == 'L') cmdClearStats();
  if (cmd[3] == 'E') cmdMotorTest();  
  if (cmd[3] == 'Q') cmdMotorPlot();  
  if (cmd[3] == 'O') cmdObstacle();  
  if (cmd[3] == 'F') cmdSensorTest(); 
  if (cmd[3] == 'G') cmdToggleGPSSolution();   // for developers
  if (cmd[3] == 'K') cmdKidnap();   // for developers
  if (cmd[3] == 'Z') cmdStressTest();   // for developers
  if (cmd[3] == 'Y') {
    if (cmd.length() <= 4){
      cmdTriggerWatchdog();   // for developers
    } else {
      if (cmd[4] == '2') cmdGNSSReboot();   // for developers
      if (cmd[4] == '3') cmdSwitchOffRobot();   // for developers
    }
  }
}

// process console input
void processConsole(){
  char ch;      
  if (CONSOLE.available()){
    battery.resetIdle();  
    while ( CONSOLE.available() ){               
      ch = CONSOLE.read();          
      if ((ch == '\r') || (ch == '\n')) {        
        CONSOLE.print("CON:");
        CONSOLE.println(cmd);
        processCmd(false, false);              
        CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}
  
// process Bluetooth input
void processBLE(){
  char ch;   
  if (BLE.available()){
    battery.resetIdle();  
    while ( BLE.available() ){    
      ch = BLE.read();      
      if ((ch == '\r') || (ch == '\n')) {   
        CONSOLE.print("BLE:");     
        CONSOLE.println(cmd);        
        processCmd(true, true);              
        BLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }    
  }  
}  

// process WIFI input (relay client)
// a relay server allows to access the robot via the Internet by transferring data from app to robot and vice versa
// client (app) --->  relay server  <--- client (robot)
void processWifiRelayClient(){
  if (!wifiFound) return;
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
        cmd = "";
        while ((wifiClient.connected()) && (wifiClient.available()) && (millis() < timeout)) {
          char ch = wifiClient.read();
          timeout = millis() + 200;
          cmd = cmd + ch;
          gps.run();
        }
        CONSOLE.print("WIF:");
        CONSOLE.println(cmd);
        if (wifiClient.connected()) {
          processCmd(true,true);
          String s = "HTTP/1.1 200 OK\r\n";
            s += "Host: " RELAY_USER "." RELAY_MACHINE "." RELAY_HOST ":";        
            s += String(RELAY_PORT) + "\r\n";
            s += "Access-Control-Allow-Origin: *\r\n";              
            s += "Content-Type: text/html\r\n";              
            s += "Connection: close\r\n";  // the connection will be closed after completion of the response
            // "Refresh: 1\r\n"        // refresh the page automatically every 20 sec                                    
            s += "Content-length: ";
            s += String(cmdResponse.length());
            s += "\r\n\r\n";  
            s += cmdResponse;                      
            wifiClient.print(s);                                   
        }
        break;
      }
    }
  }
}



// process WIFI input (App server)
// client (app) --->  server (robot)
void processWifiAppServer()
{
  if (!wifiFound) return;
  if (!ENABLE_SERVER) return;
  // listen for incoming clients    
  if (client){
    if (stopClientTime != 0) {
      if (millis() > stopClientTime){
        CONSOLE.println("app stopping client");
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
    CONSOLE.println("New client");             // print a message out the serial port
    battery.resetIdle();
    buf.init();                               // initialize the circular buffer
    unsigned long timeout = millis() + 50;
    while ( (client.connected()) && (millis() < timeout) ) {              // loop while the client's connected
      if (client.available()) {               // if there's bytes to read from the client,        
        char c = client.read();               // read a byte, then
        timeout = millis() + 50;
        buf.push(c);                          // push it to the ring buffer
        // you got two newline characters in a row
        // that's the end of the HTTP request, so send a response
        if (buf.endsWith("\r\n\r\n")) {
          cmd = "";
          while ((client.connected()) && (client.available()) && (millis() < timeout)) {
            char ch = client.read();
            timeout = millis() + 50;
            cmd = cmd + ch;
            gps.run();
          }
          CONSOLE.print("WIF:");
          CONSOLE.println(cmd);
          if (client.connected()) {
            processCmd(true,true);
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
          }
          break;
        }
      } 
      gps.run();
    }    
    // give the web browser time to receive the data
    stopClientTime = millis() + 100;
    //delay(10);
    // close the connection
    //client.stop();
    //CONSOLE.println("Client disconnected");
  }                  
}


void mqttReconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    CONSOLE.println("MQTT: Attempting connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      CONSOLE.println("MQTT: connected");
      // Once connected, publish an announcement...
      //mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      CONSOLE.println("MQTT: subscribing " MQTT_TOPIC_PREFIX "/cmd");
      mqttClient.subscribe(MQTT_TOPIC_PREFIX "/cmd");
    } else {
      CONSOLE.print("MQTT: failed, rc=");
      CONSOLE.print(mqttClient.state());
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  CONSOLE.print("MQTT: Message arrived [");
  CONSOLE.print(topic);
  CONSOLE.print("] ");
  String cmd = ""; 
  for (int i = 0; i < length; i++) {
    cmd += (char)payload[i];    
  }
  CONSOLE.println(cmd);
  if (cmd == "dock") {
    setOperation(OP_DOCK, false, true);
  } else if (cmd ==  "stop") {
    setOperation(OP_IDLE, false, true);
  } else if (cmd == "start"){
    setOperation(OP_MOW, false, true);
  }
}

// process MQTT input/output (subcriber/publisher)
void processWifiMqttClient()
{
  if (!ENABLE_MQTT) return; 
  if (millis() >= nextPublishTime){
    nextPublishTime = millis() + 10000;
    if (mqttClient.connected()) {
      updateStateOpText();
      snprintf (mqttMsg, MSG_BUFFER_SIZE, "%s", stateOpText.c_str());                
      //CONSOLE.println("MQTT: publishing " MQTT_TOPIC_PREFIX "/status");      
      mqttClient.publish(MQTT_TOPIC_PREFIX "/op", mqttMsg);      
      snprintf (mqttMsg, MSG_BUFFER_SIZE, "%.2f, %.2f", gps.relPosN, gps.relPosE);          
      mqttClient.publish(MQTT_TOPIC_PREFIX "/gps/pos", mqttMsg);
      snprintf (mqttMsg, MSG_BUFFER_SIZE, "%s", gpsSolText.c_str());          
      mqttClient.publish(MQTT_TOPIC_PREFIX "/gps/sol", mqttMsg);    
    } else {
      mqttReconnect();  
    }
  }
  mqttClient.loop();
}


void processComm(){
  processConsole();     
  processBLE();     
  processWifiAppServer();
  processWifiRelayClient();
  processWifiMqttClient();
  if (triggerWatchdog) {
    CONSOLE.println("hang test - watchdog should trigger and perform a reset");
    while (true){
      // do nothing, just hang    
    }
  }
}


// output summary on console
void outputConsole(){
  //return;
  if (millis() > nextInfoTime){        
    bool started = (nextInfoTime == 0);
    nextInfoTime = millis() + 5000;                   
    unsigned long totalsecs = millis()/1000;
    unsigned long totalmins = totalsecs/60;
    unsigned long hour = totalmins/60;
    unsigned long min = totalmins % 60;
    unsigned long sec = totalsecs % 60;
    CONSOLE.print (hour);        
    CONSOLE.print (":");    
    CONSOLE.print (min);        
    CONSOLE.print (":");    
    CONSOLE.print (sec);     
    CONSOLE.print (" ctlDur=");        
    //if (!imuIsCalibrating){
    if (!started){
      if (controlLoops > 0){
        statControlCycleTime = 1.0 / (((float)controlLoops)/5.0);
      } else statControlCycleTime = 5;
      statMaxControlCycleTime = max(statMaxControlCycleTime, statControlCycleTime);    
    }
    controlLoops=0;    
    CONSOLE.print (statControlCycleTime);        
    CONSOLE.print (" op=");    
    CONSOLE.print (stateOp);
    CONSOLE.print (" freem=");
    CONSOLE.print (freeMemory ());
    #ifndef __linux__
      uint32_t *spReg = (uint32_t*)__get_MSP();   // stack pointer
      CONSOLE.print (" sp=");
      CONSOLE.print (*spReg, HEX);
    #endif
    CONSOLE.print(" volt=");
    CONSOLE.print(battery.batteryVoltage);
    CONSOLE.print(" chg=");
    CONSOLE.print(battery.chargingCurrent);    
    CONSOLE.print(" tg=");
    CONSOLE.print(maps.targetPoint.x());
    CONSOLE.print(",");
    CONSOLE.print(maps.targetPoint.y());
    CONSOLE.print(" x=");
    CONSOLE.print(stateX);
    CONSOLE.print(" y=");
    CONSOLE.print(stateY);
    CONSOLE.print(" delta=");
    CONSOLE.print(stateDelta);    
    CONSOLE.print(" tow=");
    CONSOLE.print(gps.iTOW);
    CONSOLE.print(" lon=");
    CONSOLE.print(gps.lon,8);
    CONSOLE.print(" lat=");
    CONSOLE.print(gps.lat,8);    
    CONSOLE.print(" h=");
    CONSOLE.print(gps.height,1);    
    CONSOLE.print(" n=");
    CONSOLE.print(gps.relPosN);
    CONSOLE.print(" e=");
    CONSOLE.print(gps.relPosE);
    CONSOLE.print(" d=");
    CONSOLE.print(gps.relPosD);
    CONSOLE.print(" sol=");    
    CONSOLE.print(gps.solution);
    CONSOLE.print(" age=");    
    CONSOLE.print((millis()-gps.dgpsAge)/1000.0);
    CONSOLE.println();
    //logCPUHealth();    
  }
}
