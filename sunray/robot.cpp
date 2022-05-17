// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include <SD.h>

#include "robot.h"
#include "comm.h"
#ifdef __linux__
  #include <BridgeClient.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "PubSubClient.h"
#include "SparkFunHTU21D.h"
#include "RunningMedian.h"
#include "pinman.h"
#include "ble.h"
#include "motor.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/SerialRobotDriver.h"
#include "src/driver/MpuDriver.h"
#include "src/driver/BnoDriver.h"
#include "battery.h"
#include "gps.h"
#include "src/ublox/ublox.h"
#include "src/skytraq/skytraq.h"
#include "buzzer.h"
#include "rcmodel.h"
#include "map.h"
#include "config.h"
#include "helper.h"
#include "pid.h"
#include "reset.h"
#include "cpu.h"
#include "i2c.h"


// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

File stateFile;
#ifdef DRV_SIM_ROBOT
  SimImuDriver imuDriver(robotDriver);
#elif BNO055
  BnoDriver imuDriver;  
#else
  MpuDriver imuDriver;
#endif
#ifdef DRV_SERIAL_ROBOT
  SerialRobotDriver robotDriver;
  SerialMotorDriver motorDriver(robotDriver);
  SerialBatteryDriver batteryDriver(robotDriver);
  SerialBumperDriver bumper(robotDriver);
  SerialStopButtonDriver stopButton(robotDriver);
  SerialRainSensorDriver rainDriver(robotDriver);
  SerialLiftSensorDriver liftDriver(robotDriver);
  SerialBuzzerDriver buzzerDriver(robotDriver);
#elif DRV_SIM_ROBOT
  SimRobotDriver robotDriver;
  SimMotorDriver motorDriver(robotDriver);
  SimBatteryDriver batteryDriver(robotDriver);
  SimBumperDriver bumper(robotDriver);
  SimStopButtonDriver stopButton(robotDriver);
  SimRainSensorDriver rainDriver(robotDriver);
  SimLiftSensorDriver liftDriver(robotDriver);
  SimBuzzerDriver buzzerDriver(robotDriver);
#else
  AmRobotDriver robotDriver;
  AmMotorDriver motorDriver;
  AmBatteryDriver batteryDriver;
  AmBumperDriver bumper;
  AmStopButtonDriver stopButton;
  AmRainSensorDriver rainDriver;
  AmLiftSensorDriver liftDriver;
  AmBuzzerDriver buzzerDriver;
#endif
Motor motor;
Battery battery;
PinManager pinMan;
#ifdef DRV_SIM_ROBOT
  SimGpsDriver gps(robotDriver);
#elif GPS_SKYTRAQ
  SKYTRAQ gps;
#else 
  UBLOX gps;
#endif 
BLEConfig bleConfig;
Buzzer buzzer;
Sonar sonar;
VL53L0X tof(VL53L0X_ADDRESS_DEFAULT);
Map maps;
HTU21D myHumidity;
RCModel rcmodel;
PID pidLine(0.2, 0.01, 0); // not used
PID pidAngle(2, 0.1, 0);  // not used

int stateButton = 0;  
int stateButtonTemp = 0;
unsigned long stateButtonTimeout = 0;
OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor
unsigned long controlLoops = 0;
String stateOpText = "";  // current operation as text
String gpsSolText = ""; // current gps solution as text
float stateX = 0;  // position-east (m)
float stateY = 0;  // position-north (m)
float stateDelta = 0;  // direction (rad)
float stateRoll = 0;
float statePitch = 0;
float stateDeltaGPS = 0;
float stateDeltaIMU = 0;
float stateGroundSpeed = 0; // m/s
float stateTemp = 0; // degreeC
float stateHumidity = 0; // percent
bool stateInMotionLP = false; // robot is in angular or linear motion? (with motion low-pass filtering)
unsigned long stateInMotionLastTime = 0;
float setSpeed = 0.1; // linear speed (m/s)
unsigned long stateLeftTicks = 0;
unsigned long stateRightTicks = 0;
unsigned long lastFixTime = 0;
int fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
bool finishAndRestart = false;
bool resetLastPos = true;
bool rotateLeft = false;
bool rotateRight = false;
bool angleToTargetFits = false;
bool targetReached = false;
bool stateChargerConnected = false;
bool imuIsCalibrating = false;
int imuCalibrationSeconds = 0;
unsigned long nextImuCalibrationSecond = 0;
float lateralError = 0; // lateral error
float rollChange = 0;
float pitchChange = 0;
float lastGPSMotionX = 0;
float lastGPSMotionY = 0;
unsigned long nextGPSMotionCheckTime = 0;
bool lastMapRoutingFailed = false;
int mapRoutingFailedCounter = 0;
unsigned long retryOperationTime = 0;

SolType lastSolution = SOL_INVALID;    
unsigned long nextStatTime = 0;
unsigned long nextToFTime = 0;
unsigned long statIdleDuration = 0; // seconds
unsigned long statChargeDuration = 0; // seconds
unsigned long statMowDurationInvalid = 0; // seconds
unsigned long statMowDuration = 0; // seconds
unsigned long statMowDurationFloat = 0; // seconds
unsigned long statMowDurationFix = 0; // seconds
unsigned long statMowFloatToFixRecoveries = 0; // counter
unsigned long statMowInvalidRecoveries = 0; // counter
unsigned long statImuRecoveries = 0; // counter
unsigned long statMowObstacles = 0 ; // counter
unsigned long statMowBumperCounter = 0; 
unsigned long statMowSonarCounter = 0;
unsigned long statMowLiftCounter = 0;
unsigned long statMowGPSMotionTimeoutCounter = 0;
unsigned long statGPSJumps = 0; // counter
float statTempMin = 9999; 
float statTempMax = -9999; 
float statMowMaxDgpsAge = 0; // seconds
float statMowDistanceTraveled = 0; // meter

double stateCRC = 0;

float lastPosN = 0;
float lastPosE = 0;

unsigned long linearMotionStartTime = 0;
unsigned long angularMotionStartTime = 0;
unsigned long overallMotionTimeout = 0;
unsigned long driveReverseStopTime = 0;
unsigned long driveForwardStopTime = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextImuTime = 0;
unsigned long nextTempTime = 0;
unsigned long imuDataTimeout = 0;
unsigned long nextSaveTime = 0;
float lastIMUYaw = 0; 

bool wifiFound = false;
char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;        // your network password
WiFiEspServer server(80);
bool hasClient = false;
WiFiEspClient client;
WiFiEspClient espClient;
PubSubClient mqttClient(espClient);
//int status = WL_IDLE_STATUS;     // the Wifi radio's status
#ifdef ENABLE_NTRIP
  NTRIPClient ntrip;  // NTRIP tcp client (optional)
#endif
#ifdef GPS_USE_TCP
  WiFiClient gpsClient; // GPS tcp client (optional)  
#endif

float dockSignal = 0;
float dockAngularSpeed = 0.1;
bool dockingInitiatedByOperator = false;
bool gpsJump = false;
int motorErrorCounter = 0;
float trackerDiffDelta = 0;
float stateDeltaLast = 0;
float stateDeltaSpeed = 0;
float stateDeltaSpeedLP = 0;
float stateDeltaSpeedIMU = 0;
float stateDeltaSpeedWheels = 0;
float diffIMUWheelYawSpeed = 0;
float diffIMUWheelYawSpeedLP = 0;
bool dockReasonRainTriggered = false;

unsigned long recoverGpsTime = 0;
int recoverGpsCounter = 0;

RunningMedian<unsigned int,3> tofMeasurements;

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;    
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;    

// must be defined to override default behavior
void watchdogSetup (void){} 


// reset linear motion measurement
void resetLinearMotionMeasurement(){
  linearMotionStartTime = millis();  
  //stateGroundSpeed = 1.0;
}

// reset angular motion measurement
void resetAngularMotionMeasurement(){
  angularMotionStartTime = millis();
}

// reset overall motion timeout
void resetOverallMotionTimeout(){
  overallMotionTimeout = millis() + 10000;      
}

void updateGPSMotionCheckTime(){
  nextGPSMotionCheckTime = millis() + GPS_MOTION_DETECTION_TIMEOUT * 1000;     
}


void dumpState(){
  CONSOLE.print("dumpState: ");
  CONSOLE.print(" X=");
  CONSOLE.print(stateX);
  CONSOLE.print(" Y=");
  CONSOLE.print(stateY);
  CONSOLE.print(" delta=");
  CONSOLE.print(stateDelta);
  CONSOLE.print(" mapCRC=");
  CONSOLE.print(maps.mapCRC);
  CONSOLE.print(" mowPointsIdx=");
  CONSOLE.print(maps.mowPointsIdx);
  CONSOLE.print(" dockPointsIdx=");
  CONSOLE.print(maps.freePointsIdx);
  CONSOLE.print(" freePointsIdx=");
  CONSOLE.print(maps.freePointsIdx);
  CONSOLE.print(" wayMode=");
  CONSOLE.print(maps.wayMode);
  CONSOLE.print(" op=");
  CONSOLE.print(stateOp);
  CONSOLE.print(" sensor=");
  CONSOLE.print(stateSensor);
  CONSOLE.print(" sonar.enabled=");
  CONSOLE.print(sonar.enabled);
  CONSOLE.print(" fixTimeout=");
  CONSOLE.print(fixTimeout);
  CONSOLE.print(" absolutePosSource=");
  CONSOLE.print(absolutePosSource);
  CONSOLE.print(" lon=");
  CONSOLE.print(absolutePosSourceLon);
  CONSOLE.print(" lat=");
  CONSOLE.println(absolutePosSourceLat);
}

void updateStateOpText(){
  switch (stateOp){
    case OP_IDLE: stateOpText = "idle"; break;
    case OP_MOW: stateOpText = "mow"; break;
    case OP_CHARGE: stateOpText = "charge"; break;
    case OP_ERROR: 
      stateOpText = "error (";
      switch (stateSensor){
        case SENS_NONE: stateOpText += "none)"; break;
        case SENS_BAT_UNDERVOLTAGE: stateOpText += "unvervoltage)"; break;            
        case SENS_OBSTACLE: stateOpText += "obstacle)"; break;      
        case SENS_GPS_FIX_TIMEOUT: stateOpText += "fix timeout)"; break;
        case SENS_IMU_TIMEOUT: stateOpText += "imu timeout)"; break;
        case SENS_IMU_TILT: stateOpText += "imu tilt)"; break;
        case SENS_KIDNAPPED: stateOpText += "kidnapped)"; break;
        case SENS_OVERLOAD: stateOpText += "overload)"; break;
        case SENS_MOTOR_ERROR: stateOpText += "motor error)"; break;
        case SENS_GPS_INVALID: stateOpText += "gps invalid)"; break;
        case SENS_ODOMETRY_ERROR: stateOpText += "odo error)"; break;
        case SENS_MAP_NO_ROUTE: stateOpText += "no map route)"; break;
        case SENS_MEM_OVERFLOW: stateOpText += "mem overflow)"; break;
        case SENS_BUMPER: stateOpText += "bumper)"; break;
        case SENS_SONAR: stateOpText += "sonar)"; break;
        case SENS_LIFT: stateOpText += "lift)"; break;
        case SENS_RAIN: stateOpText += "rain)"; break;
        case SENS_STOP_BUTTON: stateOpText += "stop button)"; break;
        default: stateOpText += "unknown)"; break;
      }
      break;
    case OP_DOCK: stateOpText = "dock"; break;
    default: stateOpText = "unknown"; break;
  }
  switch (gps.solution){
    case SOL_INVALID: gpsSolText = "invalid"; break;
    case SOL_FLOAT: gpsSolText = "float"; break;
    case SOL_FIXED: gpsSolText ="fixed"; break;
    default: gpsSolText = "unknown";      
  }
}

double calcStateCRC(){
 return (stateOp *10 + maps.mowPointsIdx + maps.dockPointsIdx + maps.freePointsIdx + ((byte)maps.wayMode) 
   + sonar.enabled + fixTimeout 
   + ((byte)absolutePosSource) + absolutePosSourceLon + absolutePosSourceLat);
}

bool loadState(){
#if defined(ENABLE_SD_RESUME)
  CONSOLE.println("resuming is activated");
  CONSOLE.print("state load... ");
  if (!SD.exists("state.bin")) {
    CONSOLE.println("no state file!");
    return false;
  }
  stateFile = SD.open("state.bin", FILE_READ);
  if (!stateFile){        
    CONSOLE.println("ERROR opening file for reading");
    return false;
  }
  uint32_t marker = 0;
  stateFile.read((uint8_t*)&marker, sizeof(marker));
  if (marker != 0x10001003){
    CONSOLE.print("ERROR: invalid marker: ");
    CONSOLE.println(marker, HEX);
    return false;
  }
  long crc = 0;
  stateFile.read((uint8_t*)&crc, sizeof(crc));
  if (crc != maps.mapCRC){
    CONSOLE.print("ERROR: non-matching map CRC:");
    CONSOLE.print(crc);
    CONSOLE.print(" expected: ");
    CONSOLE.println(maps.mapCRC);
    return false;
  }
  bool res = true;
  OperationType savedOp;
  res &= (stateFile.read((uint8_t*)&stateX, sizeof(stateX)) != 0);
  res &= (stateFile.read((uint8_t*)&stateY, sizeof(stateY)) != 0);
  res &= (stateFile.read((uint8_t*)&stateDelta, sizeof(stateDelta)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.mowPointsIdx, sizeof(maps.mowPointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.dockPointsIdx, sizeof(maps.dockPointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.freePointsIdx, sizeof(maps.freePointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.wayMode, sizeof(maps.wayMode)) != 0);
  res &= (stateFile.read((uint8_t*)&savedOp, sizeof(savedOp)) != 0);
  res &= (stateFile.read((uint8_t*)&stateSensor, sizeof(stateSensor)) != 0);
  res &= (stateFile.read((uint8_t*)&sonar.enabled, sizeof(sonar.enabled)) != 0);
  res &= (stateFile.read((uint8_t*)&fixTimeout, sizeof(fixTimeout)) != 0);
  res &= (stateFile.read((uint8_t*)&setSpeed, sizeof(setSpeed)) != 0);
  res &= (stateFile.read((uint8_t*)&absolutePosSource, sizeof(absolutePosSource)) != 0);
  res &= (stateFile.read((uint8_t*)&absolutePosSourceLon, sizeof(absolutePosSourceLon)) != 0);
  res &= (stateFile.read((uint8_t*)&absolutePosSourceLat, sizeof(absolutePosSourceLat)) != 0); 
  stateFile.close();  
  CONSOLE.println("ok");
  stateCRC = calcStateCRC();
  dumpState();
  if (getResetCause() == RST_WATCHDOG){
    CONSOLE.println("resuming operation due to watchdog trigger");
    stateOp = savedOp;
    setOperation(stateOp, true, true);
  }
#endif
  return true;
}


bool saveState(){   
  bool res = true;
#if defined(ENABLE_SD_RESUME)
  double crc = calcStateCRC();
  //CONSOLE.print("stateCRC=");
  //CONSOLE.print(stateCRC);
  //CONSOLE.print(" crc=");
  //CONSOLE.println(crc);
  if (crc == stateCRC) return true;
  stateCRC = crc;
  dumpState();
  CONSOLE.print("save state... ");
  stateFile = SD.open("state.bin",  FILE_CREATE); // O_WRITE | O_CREAT);
  if (!stateFile){        
    CONSOLE.println("ERROR opening file for writing");
    return false;
  }
  uint32_t marker = 0x10001003;
  res &= (stateFile.write((uint8_t*)&marker, sizeof(marker)) != 0); 
  res &= (stateFile.write((uint8_t*)&maps.mapCRC, sizeof(maps.mapCRC)) != 0); 

  res &= (stateFile.write((uint8_t*)&stateX, sizeof(stateX)) != 0);
  res &= (stateFile.write((uint8_t*)&stateY, sizeof(stateY)) != 0);
  res &= (stateFile.write((uint8_t*)&stateDelta, sizeof(stateDelta)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.mowPointsIdx, sizeof(maps.mowPointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.dockPointsIdx, sizeof(maps.dockPointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.freePointsIdx, sizeof(maps.freePointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.wayMode, sizeof(maps.wayMode)) != 0);
  res &= (stateFile.write((uint8_t*)&stateOp, sizeof(stateOp)) != 0);
  res &= (stateFile.write((uint8_t*)&stateSensor, sizeof(stateSensor)) != 0);
  res &= (stateFile.write((uint8_t*)&sonar.enabled, sizeof(sonar.enabled)) != 0);
  res &= (stateFile.write((uint8_t*)&fixTimeout, sizeof(fixTimeout)) != 0);
  res &= (stateFile.write((uint8_t*)&setSpeed, sizeof(setSpeed)) != 0);
  res &= (stateFile.write((uint8_t*)&absolutePosSource, sizeof(absolutePosSource)) != 0);
  res &= (stateFile.write((uint8_t*)&absolutePosSourceLon, sizeof(absolutePosSourceLon)) != 0);
  res &= (stateFile.write((uint8_t*)&absolutePosSourceLat, sizeof(absolutePosSourceLat)) != 0);
  if (res){
    CONSOLE.println("ok");
  } else {
    CONSOLE.println("ERROR saving state");
  }
  stateFile.flush();
  stateFile.close();
#endif
  return res; 
}


void sensorTest(){
  CONSOLE.println("testing sensors for 60 seconds...");
  unsigned long stopTime = millis() + 60000;  
  unsigned long nextMeasureTime = 0;
  while (millis() < stopTime){
    sonar.run();
    bumper.run();
    if (millis() > nextMeasureTime){
      nextMeasureTime = millis() + 1000;      
      if (SONAR_ENABLE){
        CONSOLE.print("sonar (enabled,left,center,right,triggered): ");
        CONSOLE.print(sonar.enabled);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceLeft);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceCenter);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceRight);
        CONSOLE.print("\t");
        CONSOLE.print(((int)sonar.obstacle()));
        CONSOLE.print("\t");
      }
      if (TOF_ENABLE){   
        CONSOLE.print("ToF (dist): ");
        int v = tof.readRangeContinuousMillimeters();        
        if (!tof.timeoutOccurred()) {     
          CONSOLE.print(v/10);
        }
        CONSOLE.print("\t");
      }    
      if (BUMPER_ENABLE){
        CONSOLE.print("bumper (triggered): ");
        CONSOLE.print(((int)bumper.obstacle()));
        CONSOLE.print("\t");
       
      } 
      CONSOLE.println();  
      watchdogReset();
      robotDriver.run();   
    }
  }
  CONSOLE.println("end of sensor test - please ignore any IMU/GPS errors");
}


void startWIFI(){
#ifdef __linux__
  wifiFound = true;
#else  
  CONSOLE.println("probing for ESP8266 (NOTE: will fail for ESP32)...");
  int status = WL_IDLE_STATUS;     // the Wifi radio's status
  WIFI.begin(WIFI_BAUDRATE); 
  WIFI.print("AT\r\n");  
  delay(500);
  String res = "";  
  while (WIFI.available()){
    char ch = WIFI.read();    
    res += ch;
  }
  if (res.indexOf("OK") == -1){
    CONSOLE.println("WIFI (ESP8266) not found! If you have ESP8266 and the problem persist, you may need to flash your ESP to firmware 2.2.1");
    return;
  }    
  WiFi.init(&WIFI);  
  if (WiFi.status() == WL_NO_SHIELD) {
    CONSOLE.println("ERROR: WiFi not present");       
    return;
  }   
  wifiFound = true;
  CONSOLE.print("WiFi found! ESP8266 firmware: ");
  CONSOLE.println(WiFi.firmwareVersion());       
  if (START_AP){
    CONSOLE.print("Attempting to start AP ");  
    CONSOLE.println(ssid);
    // uncomment these two lines if you want to set the IP address of the AP
    #ifdef WIFI_IP  
      IPAddress localIp(WIFI_IP);
      WiFi.configAP(localIp);  
    #endif            
    // start access point
    status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);         
  } else {
    while ( status != WL_CONNECTED) {
      CONSOLE.print("Attempting to connect to WPA SSID: ");
      CONSOLE.println(ssid);      
      status = WiFi.begin(ssid, pass);
      #ifdef WIFI_IP  
        IPAddress localIp(WIFI_IP);
        WiFi.config(localIp);  
      #endif
    }    
  } 
  CONSOLE.print("You're connected with SSID=");    
  CONSOLE.print(WiFi.SSID());
  CONSOLE.print(" and IP=");        
  IPAddress ip = WiFi.localIP();    
  CONSOLE.println(ip);   
#endif         
  #if defined(ENABLE_UDP)
    udpSerial.beginUDP();  
  #endif    
  if (ENABLE_SERVER){
    //server.listenOnLocalhost();
    server.begin();
  }
  if (ENABLE_MQTT){
    CONSOLE.println("MQTT: enabled");
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
  }  
}


// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
bool startIMU(bool forceIMU){    
  // detect IMU
  uint8_t data = 0;
  int counter = 0;  
  while ((forceIMU) || (counter < 1)){          
     imuDriver.detect();
     if (imuDriver.imuFound){
       break;
     }
     I2Creset();  
     Wire.begin();    
     #ifdef I2C_SPEED
       Wire.setClock(I2C_SPEED);   
     #endif
     counter++;
     if (counter > 5){    
       // no I2C recovery possible - this should not happen (I2C module error)
       CONSOLE.println("ERROR IMU not found");
       stateSensor = SENS_IMU_TIMEOUT;
       setOperation(OP_ERROR);      
       //buzzer.sound(SND_STUCK, true);            
       return false;
     }
     watchdogReset();          
  }  
  if (!imuDriver.imuFound) return false;  
  counter = 0;  
  while (true){    
    if (imuDriver.begin()) break;
    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.print("Check connections, and try again.");
    CONSOLE.println();
    delay(1000);    
    counter++;
    if (counter > 5){
      stateSensor = SENS_IMU_TIMEOUT;
      setOperation(OP_ERROR);      
      //buzzer.sound(SND_STUCK, true);            
      return false;
    }
    watchdogReset();     
  }              
  imuIsCalibrating = true;   
  nextImuCalibrationSecond = millis() + 1000;
  imuCalibrationSeconds = 0;
  return true;
}


// read IMU sensor (and restart if required)
// I2C recovery: It can be minutes or hours, then there's an I2C error (probably due an spike on the 
// SCL/SDA lines) and the I2C bus on the pcb1.3 (and the arduino library) hangs and communication is delayed. 
// We check if the communication is significantly (10ms instead of 1ms) delayed, if so we restart the I2C 
// bus (by clocking out any garbage on the I2C bus) and then restarting the IMU module.
// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/using-the-mpu-9250-dmp-arduino-library
void readIMU(){
  if (!imuDriver.imuFound) return;
  // Check for new data in the FIFO  
  unsigned long startTime = millis();
  bool avail = (imuDriver.isDataAvail());
  // check time for I2C access : if too long, there's an I2C issue and we need to restart I2C bus...
  unsigned long duration = millis() - startTime;    
  //CONSOLE.print("duration:");
  //CONSOLE.println(duration);  
  if ((duration > 10) || (millis() > imuDataTimeout)) {
    if (millis() > imuDataTimeout){
      CONSOLE.println("ERROR IMU data timeout (check RTC battery if problem persists)");  
    } else {
      CONSOLE.print("ERROR IMU timeout: ");
      CONSOLE.print(duration);     
      CONSOLE.println(" (check RTC battery if problem persists)");          
    }
    stateSensor = SENS_IMU_TIMEOUT;
    motor.stopImmediately(true);    
    statImuRecoveries++;            
    if (!startIMU(true)){ // restart I2C bus
      return;
    }    
    return;
  } 
  
  if (avail) {        
    //CONSOLE.println("fifoAvailable");
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    #ifdef ENABLE_TILT_DETECTION
      rollChange += (imuDriver.roll-stateRoll);
      pitchChange += (imuDriver.pitch-statePitch);               
      rollChange = 0.95 * rollChange;
      pitchChange = 0.95 * pitchChange;
      statePitch = imuDriver.pitch;
      stateRoll = imuDriver.roll;        
      //CONSOLE.print(rollChange/PI*180.0);
      //CONSOLE.print(",");
      //CONSOLE.println(pitchChange/PI*180.0);
      if ( (fabs(scalePI(imuDriver.roll)) > 60.0/180.0*PI) || (fabs(scalePI(imuDriver.pitch)) > 100.0/180.0*PI)
            || (fabs(rollChange) > 30.0/180.0*PI) || (fabs(pitchChange) > 60.0/180.0*PI)   )  {
        CONSOLE.println("ERROR IMU tilt");
        CONSOLE.print("imu ypr=");
        CONSOLE.print(imuDriver.yaw/PI*180.0);
        CONSOLE.print(",");
        CONSOLE.print(imuDriver.pitch/PI*180.0);
        CONSOLE.print(",");
        CONSOLE.print(imuDriver.roll/PI*180.0);
        CONSOLE.print(" rollChange=");
        CONSOLE.print(rollChange/PI*180.0);
        CONSOLE.print(" pitchChange=");
        CONSOLE.println(pitchChange/PI*180.0);
        stateSensor = SENS_IMU_TILT;
        setOperation(OP_ERROR);
      }           
    #endif
    motor.robotPitch = scalePI(imuDriver.pitch);
    imuDriver.yaw = scalePI(imuDriver.yaw);
    //CONSOLE.println(imuDriver.yaw / PI * 180.0);
    lastIMUYaw = scalePI(lastIMUYaw);
    lastIMUYaw = scalePIangles(lastIMUYaw, imuDriver.yaw);
    stateDeltaIMU = -scalePI ( distancePI(imuDriver.yaw, lastIMUYaw) );  
    //CONSOLE.print(imuDriver.yaw);
    //CONSOLE.print(",");
    //CONSOLE.print(stateDeltaIMU/PI*180.0);
    //CONSOLE.println();
    lastIMUYaw = imuDriver.yaw;      
    imuDataTimeout = millis() + 10000;         
  }     
}

// check for RTC module
bool checkAT24C32() {
  byte b = 0;
  int r = 0;
  unsigned int address = 0;
  Wire.beginTransmission(AT24C32_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(AT24C32_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(AT24C32_ADDRESS, 1);
      while (Wire.available() > 0 && r < 1) {        
        b = (byte)Wire.read();        
        r++;
      }
    }
  }
  #ifdef __linux__  
    return true;
  #else
    return (r == 1);
  #endif
}


void outputConfig(){
  #ifdef ENABLE_PASS
    CONSOLE.println("ENABLE_PASS");
  #endif 
  #ifdef ENABLE_TILT_DETECTION
    CONSOLE.println("ENABLE_TILT_DETECTION");
  #endif
  CONSOLE.print("FREEWHEEL_IS_AT_BACKSIDE: ");
  CONSOLE.println(FREEWHEEL_IS_AT_BACKSIDE);
  CONSOLE.print("WHEEL_BASE_CM: ");
  CONSOLE.println(WHEEL_BASE_CM);
  CONSOLE.print("WHEEL_DIAMETER: ");
  CONSOLE.println(WHEEL_DIAMETER);
  #ifdef ENABLE_LIFT_DETECTION
    CONSOLE.println("ENABLE_LIFT_DETECTION");
    #ifdef LIFT_OBSTACLE_AVOIDANCE
      CONSOLE.println("LIFT_OBSTACLE_AVOIDANCE");
    #endif
  #endif
  CONSOLE.print("ENABLE_ODOMETRY_ERROR_DETECTION: ");
  CONSOLE.println(ENABLE_ODOMETRY_ERROR_DETECTION);
  CONSOLE.print("TICKS_PER_REVOLUTION: ");
  CONSOLE.println(TICKS_PER_REVOLUTION);
  #ifdef MOTOR_DRIVER_BRUSHLESS
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_A4931");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_A4931");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308");
  #endif 
  
  CONSOLE.print("MOTOR_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOTOR_OVERLOAD_CURRENT);
  CONSOLE.print("USE_LINEAR_SPEED_RAMP: ");
  CONSOLE.println(USE_LINEAR_SPEED_RAMP);
  CONSOLE.print("MOTOR_PID_KP: ");
  CONSOLE.println(MOTOR_PID_KP);
  CONSOLE.print("MOTOR_PID_KI: ");
  CONSOLE.println(MOTOR_PID_KI);
  CONSOLE.print("MOTOR_PID_KD: ");
  CONSOLE.println(MOTOR_PID_KD);
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_LEFT_SWAP_DIRECTION");
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_RIGHT_SWAP_DIRECTION");
  #endif
  CONSOLE.print("ENABLE_DYNAMIC_MOWER_SPEED: ");
  CONSOLE.println(ENABLE_DYNAMIC_MOWER_SPEED);
  CONSOLE.print("MOW_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOW_OVERLOAD_CURRENT);
  CONSOLE.print("ENABLE_OVERLOAD_DETECTION: ");
  CONSOLE.println(ENABLE_OVERLOAD_DETECTION);
  CONSOLE.print("ENABLE_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_FAULT_DETECTION);
  CONSOLE.print("ENABLE_FAULT_OBSTACLE_AVOIDANCE: ");
  CONSOLE.println(ENABLE_FAULT_OBSTACLE_AVOIDANCE);
  CONSOLE.print("ENABLE_DYNAMIC_MOWMOTOR: ");
  CONSOLE.println(ENABLE_DYNAMIC_MOWMOTOR);
  #ifdef SONAR_INSTALLED
    CONSOLE.println("SONAR_INSTALLED");
    CONSOLE.print("SONAR_ENABLE: ");  
    CONSOLE.println(SONAR_ENABLE);
    CONSOLE.print("SONAR_TRIGGER_OBSTACLES: ");
    CONSOLE.println(SONAR_TRIGGER_OBSTACLES);
  #endif
  CONSOLE.print("RAIN_ENABLE: ");
  CONSOLE.println(RAIN_ENABLE);
  CONSOLE.print("BUMPER_ENABLE: ");
  CONSOLE.println(BUMPER_ENABLE);
  CONSOLE.print("CURRENT_FACTOR: ");
  CONSOLE.println(CURRENT_FACTOR);
  CONSOLE.print("GO_HOME_VOLTAGE: ");
  CONSOLE.println(GO_HOME_VOLTAGE);
  CONSOLE.print("BAT_FULL_VOLTAGE: ");
  CONSOLE.println(BAT_FULL_VOLTAGE);
  CONSOLE.print("BAT_FULL_CURRENT: ");
  CONSOLE.println(BAT_FULL_CURRENT);
  CONSOLE.print("BAT_SWITCH_OFF_IDLE: ");
  CONSOLE.println(BAT_SWITCH_OFF_IDLE);
  CONSOLE.print("BAT_SWITCH_OFF_UNDERVOLTAGE: ");
  CONSOLE.println(BAT_SWITCH_OFF_UNDERVOLTAGE);
  #ifdef GPS_USE_TCP
    CONSOLE.println("GPS_USE_TCP");
  #endif
  #ifdef GPS_SKYTRAQ
    CONSOLE.println("GPS_USE_SKYTRAQ");  
  #endif
  CONSOLE.print("REQUIRE_VALID_GPS: ");
  CONSOLE.println(REQUIRE_VALID_GPS);
  CONSOLE.print("GPS_SPEED_DETECTION: ");
  CONSOLE.println(GPS_SPEED_DETECTION);
  CONSOLE.print("GPS_MOTION_DETECTION: ");
  CONSOLE.println(GPS_MOTION_DETECTION);
  CONSOLE.print("GPS_REBOOT_RECOVERY: ");
  CONSOLE.println(GPS_REBOOT_RECOVERY);
  CONSOLE.print("GPS_CONFIG: ");
  CONSOLE.println(GPS_CONFIG);
  CONSOLE.print("GPS_CONFIG_FILTER: ");
  CONSOLE.println(GPS_CONFIG_FILTER);
  CONSOLE.print("CPG_CONFIG_FILTER_MINELEV: ");
  CONSOLE.println(CPG_CONFIG_FILTER_MINELEV);
  CONSOLE.print("CPG_CONFIG_FILTER_NCNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_NCNOTHRS);
  CONSOLE.print("CPG_CONFIG_FILTER_CNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_CNOTHRS);
  CONSOLE.print("ALLOW_ROUTE_OUTSIDE_PERI_METER: ");
  CONSOLE.println(ALLOW_ROUTE_OUTSIDE_PERI_METER);
  CONSOLE.print("OBSTACLE_DETECTION_ROTATION: ");
  CONSOLE.println(OBSTACLE_DETECTION_ROTATION);
  CONSOLE.print("KIDNAP_DETECT: ");
  CONSOLE.println(KIDNAP_DETECT);
  CONSOLE.print("KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE: ");
  CONSOLE.println(KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE);
  CONSOLE.print("DOCKING_STATION: ");
  CONSOLE.println(DOCKING_STATION);
  CONSOLE.print("DOCK_IGNORE_GPS: ");
  CONSOLE.println(DOCK_IGNORE_GPS);
  CONSOLE.print("DOCK_AUTO_START: ");
  CONSOLE.println(DOCK_AUTO_START);
  CONSOLE.print("STANLEY_CONTROL_P_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_P_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_K_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_K_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_P_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_P_SLOW);
  CONSOLE.print("STANLEY_CONTROL_K_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_K_SLOW);
  CONSOLE.print("BUTTON_CONTROL: ");
  CONSOLE.println(BUTTON_CONTROL);
  #ifdef USE_TEMP_SENSOR
    CONSOLE.println("USE_TEMP_SENSOR: ");
  #endif
  #ifdef BUZZER_ENABLE
    CONSOLE.println("BUZZER_ENABLE");    
  #endif
}


// robot start routine
void start(){    
  pinMan.begin();         
  // keep battery switched ON
  batteryDriver.begin();  
  CONSOLE.begin(CONSOLE_BAUDRATE);    
  buzzerDriver.begin();
  buzzer.begin();      
    
  Wire.begin();      
  analogReadResolution(12);  // configure ADC 12 bit resolution
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout){
    if (!checkAT24C32()){
      CONSOLE.println(F("PCB not powered ON or RTC module missing"));      
      I2Creset();  
      Wire.begin();    
      #ifdef I2C_SPEED
        Wire.setClock(I2C_SPEED);     
      #endif
    } else break;
  }  
  delay(1500);
    
  #if defined(ENABLE_SD)
    #ifdef __linux__
      bool res = SD.begin();
    #else 
      bool res = SD.begin(SDCARD_SS_PIN);
    #endif    
    if (res){
      CONSOLE.println("SD card found!");
      #if defined(ENABLE_SD_LOG)        
        sdSerial.beginSD();  
      #endif
    } else {
      CONSOLE.println("no SD card found");                
    }    
  #endif 
  
  logResetCause();
  
  CONSOLE.println(VER);          
  CONSOLE.print("compiled for: ");
  CONSOLE.println(BOARD);
  
  robotDriver.begin();
  CONSOLE.print("robot id: ");
  String rid = "";
  robotDriver.getRobotID(rid);
  CONSOLE.println(rid);
  motorDriver.begin();
  rainDriver.begin();
  liftDriver.begin();  
  battery.begin();      
  stopButton.begin();

  bleConfig.run();   
  //BLE.println(VER); is this needed? can confuse BLE modules if not connected?  
    
  rcmodel.begin();  
  motor.begin();
  sonar.begin();
  bumper.begin();

  outputConfig();

  if (TOF_ENABLE){
    tof.setTimeout(500);
    if (!tof.init())
    {
      CONSOLE.println("Failed to detect and initialize tof sensor");
      delay(1000);
    }
    tof.startContinuous(100);
  }        
  
  CONSOLE.print("SERIAL_BUFFER_SIZE=");
  CONSOLE.print(SERIAL_BUFFER_SIZE);
  CONSOLE.println(" (increase if you experience GPS checksum errors)");
  //CONSOLE.println("-----------------------------------------------------");
  //CONSOLE.println("NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:");
  //CONSOLE.println("1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom");
  //CONSOLE.println("2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h");
  //CONSOLE.println("   for Grand Central M4 'packages/adafruit/hardware/samd/xxxxx/cores/arduino/RingBuffer.h");  
  //CONSOLE.println("change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024");
  CONSOLE.println("-----------------------------------------------------");
  
  #ifdef GPS_USE_TCP
    gps.begin(gpsClient, GPS_HOST, GPS_PORT);
  #else 
    gps.begin(GPS, GPS_BAUDRATE);   
  #endif

  maps.begin();      
  //maps.clipperTest();
  
  myHumidity.begin();    
  
  // initialize ESP module
  startWIFI();
  #ifdef ENABLE_NTRIP
    ntrip.begin();  
  #endif
  
  watchdogEnable(10000L);   // 10 seconds  
  
  startIMU(false);        
  
  buzzer.sound(SND_READY);  
  battery.resetIdle();        
  loadState();
}


// calculate statistics
void calcStats(){
  if (millis() >= nextStatTime){
    nextStatTime = millis() + 1000;
    switch (stateOp){
      case OP_IDLE:
        statIdleDuration++;
        break;
      case OP_MOW:      
        statMowDuration++;
        if (gps.solution == SOL_FIXED) statMowDurationFix++;
          else if (gps.solution == SOL_FLOAT) statMowDurationFloat++;   
          else if (gps.solution == SOL_INVALID) statMowDurationInvalid++;
        if (gps.solution != lastSolution){      
          if ((lastSolution == SOL_FLOAT) && (gps.solution == SOL_FIXED)) statMowFloatToFixRecoveries++;
          if (lastSolution == SOL_INVALID) statMowInvalidRecoveries++;
          lastSolution = gps.solution;
        } 
        statMowMaxDgpsAge = max(statMowMaxDgpsAge, (millis() - gps.dgpsAge)/1000.0);        
        break;
      case OP_CHARGE:
        statChargeDuration++;
        break;
    }     
  }   
}


// compute robot state (x,y,delta)
// uses complementary filter ( https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/ )
// to fusion GPS heading (long-term) and IMU heading (short-term)
// with IMU: heading (stateDelta) is computed by gyro (stateDeltaIMU)
// without IMU: heading (stateDelta) is computed by odometry (deltaOdometry)
void computeRobotState(){  
  long leftDelta = motor.motorLeftTicks-stateLeftTicks;
  long rightDelta = motor.motorRightTicks-stateRightTicks;  
  stateLeftTicks = motor.motorLeftTicks;
  stateRightTicks = motor.motorRightTicks;    
    
  float distLeft = ((float)leftDelta) / ((float)motor.ticksPerCm);
  float distRight = ((float)rightDelta) / ((float)motor.ticksPerCm);  
  float distOdometry = (distLeft + distRight) / 2.0;
  float deltaOdometry = -(distLeft - distRight) / motor.wheelBaseCm;    
  
  float posN = 0;
  float posE = 0;
  if (absolutePosSource){
    relativeLL(absolutePosSourceLat, absolutePosSourceLon, gps.lat, gps.lon, posN, posE);    
  } else {
    posN = gps.relPosN;  
    posE = gps.relPosE;     
  }   
  
  if (fabs(motor.linearSpeedSet) < 0.001){       
    resetLastPos = true;
  }
  
  if ((gps.solutionAvail) 
      && ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT))  )
  {
    gps.solutionAvail = false;        
    stateGroundSpeed = 0.9 * stateGroundSpeed + 0.1 * abs(gps.groundSpeed);    
    //CONSOLE.println(stateGroundSpeed);
    float distGPS = sqrt( sq(posN-lastPosN)+sq(posE-lastPosE) );
    if ((distGPS > 0.3) || (resetLastPos)){
      if (distGPS > 0.3) {
        gpsJump = true;
        statGPSJumps++;
        CONSOLE.print("GPS jump: ");
        CONSOLE.println(distGPS);
      }
      resetLastPos = false;
      lastPosN = posN;
      lastPosE = posE;
    } else if (distGPS > 0.1) {       
      if ( (fabs(motor.linearSpeedSet) > 0) && (fabs(motor.angularSpeedSet) /PI *180.0 < 45) ) {  
        stateDeltaGPS = scalePI(atan2(posN-lastPosN, posE-lastPosE));    
        if (motor.linearSpeedSet < 0) stateDeltaGPS = scalePI(stateDeltaGPS + PI); // consider if driving reverse
        //stateDeltaGPS = scalePI(2*PI-gps.heading+PI/2);
        float diffDelta = distancePI(stateDelta, stateDeltaGPS);                 
        if (    ((gps.solution == SOL_FIXED) && (maps.useGPSfixForDeltaEstimation ))
             || ((gps.solution == SOL_FLOAT) && (maps.useGPSfloatForDeltaEstimation)) )
        {   // allows planner to use float solution?         
          if (fabs(diffDelta/PI*180) > 45){ // IMU-based heading too far away => use GPS heading
            stateDelta = stateDeltaGPS;
            stateDeltaIMU = 0;
          } else {
            // delta fusion (complementary filter, see above comment)
            stateDeltaGPS = scalePIangles(stateDeltaGPS, stateDelta);
            stateDelta = scalePI(fusionPI(0.9, stateDelta, stateDeltaGPS));               
          }            
        }
      }
      lastPosN = posN;
      lastPosE = posE;
    } 
    if (gps.solution == SOL_FIXED) {
      // fix
      lastFixTime = millis();
      if (maps.useGPSfixForPosEstimation) {
        stateX = posE;
        stateY = posN;
      }        
    } else {
      // float
      if (maps.useGPSfloatForPosEstimation){ // allows planner to use float solution?
        stateX = posE;
        stateY = posN;              
      }
    }
  } 
  
  // odometry
  stateX += distOdometry/100.0 * cos(stateDelta);
  stateY += distOdometry/100.0 * sin(stateDelta);        
  if (stateOp == OP_MOW) statMowDistanceTraveled += distOdometry/100.0;
  
  if ((imuDriver.imuFound) && (maps.useIMU)) {
    // IMU available and should be used by planner
    stateDelta = scalePI(stateDelta + stateDeltaIMU );          
  } else {
    // odometry
    stateDelta = scalePI(stateDelta + deltaOdometry);  
  }
  if (imuDriver.imuFound){
    stateDeltaSpeedIMU = 0.99 * stateDeltaSpeedIMU + 0.01 * stateDeltaIMU / 0.02; // IMU yaw rotation speed (20ms timestep)
  }
  stateDeltaSpeedWheels = 0.99 * stateDeltaSpeedWheels + 0.01 * deltaOdometry / 0.02; // wheels yaw rotation speed (20ms timestep) 
  //CONSOLE.println(stateDelta / PI * 180.0);
  stateDeltaIMU = 0;

  // compute yaw rotation speed (delta speed)
  stateDeltaSpeed = (stateDelta - stateDeltaLast) / 0.02;  // 20ms timestep
  stateDeltaSpeedLP = stateDeltaSpeedLP * 0.95 + fabs(stateDeltaSpeed) * 0.05;     
  stateDeltaLast = stateDelta;
  //CONSOLE.println(stateDeltaSpeedLP/PI*180.0);

  if (imuDriver.imuFound) {
    // compute difference between IMU yaw rotation speed and wheels yaw rotation speed
    diffIMUWheelYawSpeed = stateDeltaSpeedIMU - stateDeltaSpeedWheels;
    diffIMUWheelYawSpeedLP = diffIMUWheelYawSpeedLP * 0.95 + fabs(diffIMUWheelYawSpeed) * 0.05;  
    //CONSOLE.println(diffIMUWheelYawSpeedLP/PI*180.0);
    //CONSOLE.print(stateDeltaSpeedIMU/PI*180.0);
    //CONSOLE.print(",");
    //CONSOLE.println(stateDeltaSpeedWheels/PI*180.0);
  }
}

// should robot move?
bool robotShouldMove(){
  /*CONSOLE.print(motor.linearSpeedSet);
  CONSOLE.print(",");
  CONSOLE.println(motor.angularSpeedSet / PI * 180.0);  */
  return ( fabs(motor.linearSpeedSet) > 0.001 );
}

bool robotShouldMoveForward(){
   return ( motor.linearSpeedSet > 0.001 );
}

// should robot rotate?
bool robotShouldRotate(){
  return ( (fabs(motor.linearSpeedSet) < 0.001) &&  (fabs(motor.angularSpeedSet) > 0.001) );
}

// should robot be in motion? NOTE: function ignores very short motion pauses (with motion low-pass filtering)
bool robotShouldBeInMotion(){  
  if (robotShouldMove() || (robotShouldRotate())) {
    stateInMotionLastTime = millis();
    stateInMotionLP = true;    
  }
  if (millis() > stateInMotionLastTime + 2000) {
    stateInMotionLP = false;
  }
  return stateInMotionLP;
}


// drive reverse if robot cannot move forward
void triggerObstacle(){
  if (driveReverseStopTime != 0) return;
  CONSOLE.println("triggerObstacle");      
  statMowObstacles++;      
  if (maps.isDocking()) {    
    if (maps.retryDocking(stateX, stateY)) {
      driveReverseStopTime = millis() + 3000;                      
      return;
    }
  } 
  if ((OBSTACLE_AVOIDANCE) && (maps.wayMode != WAY_DOCK)){    
    driveReverseStopTime = millis() + 3000;      
  } else {     
    stateSensor = SENS_OBSTACLE;
    setOperation(OP_ERROR);
    buzzer.sound(SND_ERROR, true);        
  }
}


// detect sensor malfunction
void detectSensorMalfunction(){  
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if (motor.odometryError){
      CONSOLE.println("odometry error!");    
      stateSensor = SENS_ODOMETRY_ERROR;
      setOperation(OP_ERROR);
      buzzer.sound(SND_ERROR, true); 
      return;      
    }
  }
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 20000){
      CONSOLE.println("overload!");    
      stateSensor = SENS_OVERLOAD;
      setOperation(OP_ERROR);
      buzzer.sound(SND_ERROR, true);        
      return;
    }  
  }
  if (ENABLE_FAULT_OBSTACLE_AVOIDANCE){
    if (motor.motorError){
      // this is the molehole situation: motor error will permanently trigger on molehole => we try obstacle avoidance (molehole avoidance strategy)
      motor.motorError = false; // reset motor error flag
      motorErrorCounter++;       
      if (maps.wayMode != WAY_DOCK){
        if (motorErrorCounter < 5){ 
          //stateSensor = SENS_MOTOR_ERROR;
          triggerObstacle();     // trigger obstacle avoidance 
          return;
        }
      }
      // obstacle avoidance failed with too many motor errors (it was probably not a molehole situation)
      CONSOLE.println("motor error!");
      stateSensor = SENS_MOTOR_ERROR;
      setOperation(OP_ERROR);
      buzzer.sound(SND_ERROR, true);       
      return;      
    }  
  }
}

// detect lift 
// returns true, if lift detected, otherwise false
bool detectLift(){  
  #ifdef ENABLE_LIFT_DETECTION
    if (liftDriver.triggered()) {
      if (stateOp != OP_ERROR){        
        stateSensor = SENS_LIFT;
        CONSOLE.println("ERROR LIFT");        
        setOperation(OP_ERROR);
        return true;
      }      
    }  
  #endif 
  return false;
}

// detect obstacle (bumper, sonar, ToF)
// returns true, if obstacle detected, otherwise false
bool detectObstacle(){   
  if (! ((robotShouldMoveForward()) || (robotShouldRotate())) ) return false;      
  if (TOF_ENABLE){
    if (millis() >= nextToFTime){
      nextToFTime = millis() + 200;
      int v = tof.readRangeContinuousMillimeters();        
      if (!tof.timeoutOccurred()) {     
        tofMeasurements.add(v);        
        float avg = 0;
        if (tofMeasurements.getAverage(avg) == tofMeasurements.OK){
          //CONSOLE.println(avg);
          if (avg < TOF_OBSTACLE_CM * 10){
            CONSOLE.println("ToF obstacle!");    
            triggerObstacle();                
            return true; 
          }
        }      
      } 
    }    
  }   
  
  #ifdef ENABLE_LIFT_DETECTION
    #ifdef LIFT_OBSTACLE_AVOIDANCE
      if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (liftDriver.triggered()) ) {
        CONSOLE.println("lift sensor obstacle!");    
        statMowBumperCounter++;
        triggerObstacle();    
        return true;
      }
    #endif
  #endif

  if (BUMPER_ENABLE){
    if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (bumper.obstacle()) ){  
      CONSOLE.println("bumper obstacle!");    
      statMowBumperCounter++;
      triggerObstacle();    
      return true;
    }
  }
  if (sonar.obstacle() && (maps.wayMode != WAY_DOCK)){
    CONSOLE.println("sonar obstacle!");    
    statMowSonarCounter++;
    if (SONAR_TRIGGER_OBSTACLES){
      triggerObstacle();
      return true;
    }        
  }  
  // check if GPS motion (obstacle detection)  
  if ((millis() > nextGPSMotionCheckTime) || (millis() > overallMotionTimeout)) {        
    updateGPSMotionCheckTime();
    resetOverallMotionTimeout(); // this resets overall motion timeout (overall motion timeout happens if e.g. 
    // motion between anuglar-only and linar-only toggles quickly, and their specific timeouts cannot apply due to the quick toggling)
    float dX = lastGPSMotionX - stateX;
    float dY = lastGPSMotionY - stateY;
    float delta = sqrt( sq(dX) + sq(dY) );    
    if (delta < 0.05){
      if (GPS_MOTION_DETECTION){
        CONSOLE.println("gps no motion => obstacle!");
        statMowGPSMotionTimeoutCounter++;
        triggerObstacle();
        return true;
      }
    }
    lastGPSMotionX = stateX;      
    lastGPSMotionY = stateY;      
  }    
  return false;
}

// stuck rotate avoidance (drive forward if robot cannot rotate)
void triggerObstacleRotation(){
  CONSOLE.println("triggerObstacleRotation");    
  statMowObstacles++;   
  if ((OBSTACLE_AVOIDANCE) && (maps.wayMode != WAY_DOCK)){    
    if (FREEWHEEL_IS_AT_BACKSIDE){    
      driveForwardStopTime = millis() + 2000;      
    } else {
      driveReverseStopTime = millis() + 3000;
    }
  } else { 
    stateSensor = SENS_OBSTACLE;
    setOperation(OP_ERROR);
    buzzer.sound(SND_ERROR, true);        
  }
}

// stuck rotate detection (e.g. robot cannot due to an obstacle outside of robot rotation point)
// returns true, if stuck detected, otherwise false
bool detectObstacleRotation(){  
  if (!robotShouldRotate()) {
    return false;
  }  
  if (!OBSTACLE_DETECTION_ROTATION) return false; 
  if (millis() > angularMotionStartTime + 15000) { // too long rotation time (timeout), e.g. due to obstacle
    triggerObstacleRotation();
    return true;
  }
  /*if (BUMPER_ENABLE){
    if (millis() > angularMotionStartTime + 500) { // FIXME: do we actually need a deadtime here for the freewheel sensor?        
      if (bumper.obstacle()){  
        CONSOLE.println("bumper obstacle!");    
        statMowBumperCounter++;
        triggerObstacleRotation();    
        return true;
      }
    }
  }*/
  if (imuDriver.imuFound){
    if (millis() > angularMotionStartTime + 3000) {                  
      if (fabs(stateDeltaSpeedLP) < 3.0/180.0 * PI){ // less than 3 degree/s yaw speed, e.g. due to obstacle
        triggerObstacleRotation();
        return true;      
      }
    }
    if (diffIMUWheelYawSpeedLP > 10.0/180.0 * PI) {  // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle
      triggerObstacleRotation();
      return true;            
    }    
  }
  return false;
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(){  
  Point target = maps.targetPoint;
  Point lastTarget = maps.lastTargetPoint;
  float linear = 1.0;  
  bool mow = true;
  if (stateOp == OP_DOCK) mow = false;
  float angular = 0;      
  float targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);                         
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());        
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);    
  else 
    targetReached = (targetDist < 0.05);    
  
  
  if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
    linear = 0.1;  
  }   
          
  // allow rotations only near last or next waypoint or if too far away from path
  if ( (targetDist < 0.5) || (lastTargetDist < 0.5) ||  (fabs(distToPath) > 0.5) ) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 120);          
    else     
      angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 20);   
  } else angleToTargetFits = true;

               
  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = 29.0 / 180.0 * PI; //  29 degree/s (0.5 rad/s);               
    if ((!rotateLeft) && (!rotateRight)){ // decide for one rotation direction (and keep it)
      if (trackerDiffDelta < 0) rotateLeft = true;
        else rotateRight = true;
    }        
    if (rotateLeft) angular *= -1;            
    if (fabs(trackerDiffDelta)/PI*180.0 < 90){
      rotateLeft = false;  // reset rotate direction
      rotateRight = false;
    }    
  } 
  else {
    // line control (stanley)    
    bool straight = maps.nextPointIsStraight();
    if (maps.trackSlow) {
      // planner forces slow tracking (e.g. docking etc)
      linear = 0.1;           
    } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.5) && (!straight))   // approaching
          || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving  
       ) 
    {
      linear = 0.1; // reduce speed when approaching/leaving waypoints          
    } 
    else {
      if (gps.solution == SOL_FLOAT)        
        linear = min(setSpeed, 0.1); // reduce speed for float solution
      else
        linear = setSpeed;         // desired speed
      if (sonar.nearObstacle()) linear = 0.1; // slow down near obstacles
    }      
    //angula                                    r = 3.0 * trackerDiffDelta + 3.0 * lateralError;       // correct for path errors 
    float k = stanleyTrackingNormalK; // STANLEY_CONTROL_K_NORMAL;
    float p = stanleyTrackingNormalP; // STANLEY_CONTROL_P_NORMAL;    
    if (maps.trackSlow) {
      k = stanleyTrackingSlowK; //STANLEY_CONTROL_K_SLOW;   
      p = stanleyTrackingSlowP; //STANLEY_CONTROL_P_SLOW;          
    }
    angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(motor.linearSpeedSet)));       // correct for path errors           
    /*pidLine.w = 0;              
    pidLine.x = lateralError;
    pidLine.max_output = PI;
    pidLine.y_min = -PI;
    pidLine.y_max = PI;
    pidLine.compute();
    angular = -pidLine.y;   */
    //CONSOLE.print(lateralError);        
    //CONSOLE.print(",");        
    //CONSOLE.println(angular/PI*180.0);            
    if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
    if (!SMOOTH_CURVES) angular = max(-PI/16, min(PI/16, angular)); // restrict steering angle for stanley
  }
  // check some pre-conditions that can make linear+angular speed zero
  if (!maps.isUndocking()){
    if (fixTimeout != 0){
      if (millis() > lastFixTime + fixTimeout * 1000.0){
        // stop on fix solution timeout 
        linear = 0;
        angular = 0;
        mow = false; 
        stateSensor = SENS_GPS_FIX_TIMEOUT;
        //angular = 0.2;
      } else {
        //if (stateSensor == SENS_GPS_FIX_TIMEOUT) stateSensor = SENS_NONE; // clear fix timeout
      }       
    }     
  }

  if ((gps.solution == SOL_FIXED) || (gps.solution == SOL_FLOAT)){        
    if (abs(linear) > 0.06) {
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03)){
        // if in linear motion and not enough ground speed => obstacle
        //if ( (GPS_SPEED_DETECTION) && (!maps.isUndocking()) ) { 
        if (GPS_SPEED_DETECTION) {         
          CONSOLE.println("gps no speed => obstacle!");
          triggerObstacle();
          return;
        }
      }
    }  
  } else {
    // no gps solution
    if (REQUIRE_VALID_GPS){
      if (!maps.isUndocking()) { 
        CONSOLE.println("WARN: no gps solution!");
        stateSensor = SENS_GPS_INVALID;
        //setOperation(OP_ERROR);
        //buzzer.sound(SND_STUCK, true);          
        linear = 0;
        angular = 0;      
        mow = false;
      } 
    }
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT){
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;     
    if ( maps.isUndocking() ) allowedPathTolerance = 0.2;
    if (fabs(distToPath) > allowedPathTolerance){ // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      linear = 0;
      angular = 0;        
      mow = false;
      stateSensor = SENS_KIDNAPPED;
      if (millis() > recoverGpsTime){
        CONSOLE.println("KIDNAP_DETECT");
        recoverGpsTime = millis() + 30000;
        recoverGpsCounter++;
        if (recoverGpsCounter == 3){          
          setOperation(OP_ERROR);
          buzzer.sound(SND_ERROR, true);        
          return;
        }   
        if (GPS_REBOOT_RECOVERY){           
          gps.reboot();   // try to recover from false GPS fix     
        }
      }      
    } else {
      recoverGpsTime = millis() + 30000;
      recoverGpsCounter = 0;
    }
  }
   
  if (mow)  {  // wait until mowing motor is running
    if (millis() < motor.motorMowSpinUpTime + 5000){
      if (!buzzer.isPlaying()) buzzer.sound(SND_WARNING, true);
      linear = 0;
      angular = 0;   
    }
  }

  motor.setLinearAngularSpeed(linear, angular);      
  motor.setMowState(mow);

  if (targetReached){
    if (maps.wayMode == WAY_MOW){
      maps.clearObstacles(); // clear obstacles if target reached
      motorErrorCounter = 0; // reset motor error counter if target reached
      stateSensor = SENS_NONE; // clear last triggered sensor
    }
    bool straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false)){
      // finish        
      if (stateOp == OP_DOCK){
        CONSOLE.println("docking finished!");
        setOperation(OP_IDLE); 
      } else {
        CONSOLE.println("mowing finished!");
        if (!finishAndRestart){             
          if (DOCKING_STATION){
            setOperation(OP_DOCK);               
          } else {
            setOperation(OP_IDLE); 
          }
        }                   
      }
    } else {      
      // next waypoint          
      //if (!straight) angleToTargetFits = false;      
    }
  }  
}



// robot main loop
void run(){  
  #ifdef ENABLE_NTRIP
    ntrip.run();
  #endif
  robotDriver.run();
  buzzer.run();
  buzzerDriver.run();
  stopButton.run();
  battery.run();
  batteryDriver.run();
  motorDriver.run();
  rainDriver.run();
  liftDriver.run();
  motor.run();
  sonar.run();
  maps.run();  
  rcmodel.run();

  // state saving
  if (millis() >= nextSaveTime){  
    nextSaveTime = millis() + 5000;
    saveState();
  }
  
  // temp
  if (millis() > nextTempTime){
    nextTempTime = millis() + 60000;
    #ifdef USE_TEMP_SENSOR
      // https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
      stateTemp = myHumidity.readTemperature();
      statTempMin = min(statTempMin, stateTemp);
      statTempMax = max(statTempMax, stateTemp);
      stateHumidity = myHumidity.readHumidity();      
      CONSOLE.print("temp=");
      CONSOLE.print(stateTemp,1);
      CONSOLE.print("  humidity=");
      CONSOLE.print(stateHumidity,0);    
      CONSOLE.println();        
    #endif
    logCPUHealth();
    CONSOLE.println();
  }
  
  // IMU
  if (millis() > nextImuTime){
    nextImuTime = millis() + 150;        
    //imu.resetFifo();    
    if (imuIsCalibrating) {
      motor.stopImmediately(true);   
      if (millis() > nextImuCalibrationSecond){
        nextImuCalibrationSecond = millis() + 1000;  
        imuCalibrationSeconds++;
        CONSOLE.print("IMU gyro calibration (robot must be static)... ");        
        CONSOLE.println(imuCalibrationSeconds);        
        buzzer.sound(SND_PROGRESS, true);        
        if (imuCalibrationSeconds >= 9){
          imuIsCalibrating = false;
          CONSOLE.println();                
          lastIMUYaw = 0;          
          imuDriver.resetData();
          imuDataTimeout = millis() + 10000;
        }
      }       
    } else {
      readIMU();    
    }
  }
  
  gps.run();
    
  calcStats();  
  
  
  if (millis() >= nextControlTime){        
    nextControlTime = millis() + 20; 
    controlLoops++;    
    
    computeRobotState();
    if (!robotShouldMove()){
      resetLinearMotionMeasurement();
      updateGPSMotionCheckTime();  
    }
    if (!robotShouldRotate()){
      resetAngularMotionMeasurement();
    }
    if (!robotShouldBeInMotion()){
      resetOverallMotionTimeout();
      lastGPSMotionX = 0;
      lastGPSMotionY = 0;
    }

    /*if (gpsJump) {
      // gps jump: restart current operation from new position (restart path planning)
      CONSOLE.println("restarting operation (gps jump)");
      gpsJump = false;
      motor.stopImmediately(true);
      setOperation(stateOp, true);    // restart current operation
    }*/
    if (retryOperationTime != 0) {
      if (millis() > retryOperationTime){
        // restart current operation from new position (restart path planning)
        CONSOLE.println("restarting operation (retryOperationTime)");
        retryOperationTime = 0;
        motor.stopImmediately(true);
        setOperation(stateOp, true);    // restart current operation
      }
    }

    if (battery.chargerConnected() != stateChargerConnected) {    
      stateChargerConnected = battery.chargerConnected(); 
      if (stateChargerConnected){      
        // charger connected event        
        setOperation(OP_CHARGE);                
      } else {
        // charger disconnected event
        motor.enableTractionMotors(true); // allow traction motors to operate                       
        if (stateOp == OP_CHARGE) setOperation(OP_IDLE);
      }           
    } 


    // some things to do permanently while charger connected/not connected
    if (battery.chargerConnected()){
      if ((stateOp == OP_IDLE) || (stateOp == OP_CHARGE)){
        maps.setIsDocked(true);               
        // get robot position and yaw from map
        // sensing charging contacts means we are in docking station - we use docking point coordinates to get rid of false fix positions in
        // docking station
        maps.setRobotStatePosToDockingPos(stateX, stateY, stateDelta);
        // get robot yaw orientation from map 
        //float tempX;
        //float tempY;
        //maps.setRobotStatePosToDockingPos(tempX, tempY, stateDelta);                       
      }
      battery.resetIdle();        
    } else {
      if ((stateOp == OP_IDLE) || (stateOp == OP_CHARGE)){
        maps.setIsDocked(false);
      }
    }          
 
    
    if (!imuIsCalibrating){     
            
      if ((stateOp == OP_MOW) ||  (stateOp == OP_DOCK)) {              
        
        if (retryOperationTime == 0){ // if path planning was successful 
          if (driveReverseStopTime > 0){
            // obstacle avoidance
            motor.setLinearAngularSpeed(-0.1,0);
            motor.setMowState(false);                        
            if (millis() > driveReverseStopTime){
              CONSOLE.println("driveReverseStopTime");
              motor.stopImmediately(false); 
              detectLift();
              driveReverseStopTime = 0;
              if (maps.isDocking()){
                CONSOLE.println("continue docking");
                // continue without planner
              } else {
                maps.addObstacle(stateX, stateY);              
                Point pt;
                if (!maps.findObstacleSafeMowPoint(pt)){
                  setOperation(OP_DOCK, true); // dock if no more (valid) mowing points
                } else setOperation(stateOp, true);    // continue current operation
              }
            }            
          } else if (driveForwardStopTime > 0){
            // rotate stuck avoidance
            motor.setLinearAngularSpeed(0.1,0);
            motor.setMowState(false);            
            if (millis() > driveForwardStopTime){
              CONSOLE.println("driveForwardStopTime");
              motor.stopImmediately(false);  
              driveForwardStopTime = 0;
              /*maps.addObstacle(stateX, stateY);
              Point pt;
              if (!maps.findObstacleSafeMowPoint(pt)){
                setOperation(OP_DOCK, true); // dock if no more (valid) mowing points
              } else*/ setOperation(stateOp, true);    // continue current operation              
            }            
          } else {          
            // line tracking
            trackLine();
            detectSensorMalfunction();
            if (!detectObstacle()){
              detectObstacleRotation();                              
            }   
          }        
        }        
        battery.resetIdle();
        if (battery.underVoltage()){
          stateSensor = SENS_BAT_UNDERVOLTAGE;
          setOperation(OP_IDLE);
          //buzzer.sound(SND_OVERCURRENT, true);        
        } 
        if (battery.shouldGoHome()){
          if (DOCKING_STATION){
            setOperation(OP_DOCK);
          }
        }
        if (RAIN_ENABLE){
          if (rainDriver.triggered()){
            if (DOCKING_STATION){
              stateSensor = SENS_RAIN;
              dockReasonRainTriggered = true;
              setOperation(OP_DOCK);              
            }
          }
        }        
      }
      else if (stateOp == OP_CHARGE){      
        if (battery.chargerConnected()){
          if (battery.chargingHasCompleted()){
            if ((DOCKING_STATION) && (!dockingInitiatedByOperator)) {
              if (maps.mowPointsIdx > 0){  // if mowing not completed yet
                if ((DOCK_AUTO_START) && (!dockReasonRainTriggered)) { // automatic continue mowing allowed?
                  setOperation(OP_MOW); // continue mowing
                }
              }
            }
          }
        }        
      }      
      
      // process button state
      if (stateButton == 5){
        stateButton = 0; // reset button state
        stateSensor = SENS_STOP_BUTTON;
        setOperation(OP_DOCK, false, true);
      } else if (stateButton == 6){ 
          stateSensor = SENS_STOP_BUTTON;
          setOperation(OP_MOW, false, true);
      } else if (stateButton > 0){
        // stateButton 1 (or unknown button state)
        stateButton = 0;  // reset button state
        stateSensor = SENS_STOP_BUTTON;
        setOperation(OP_IDLE, false, true);                             
      }
      
      
    } // if (!imuIsCalibrating)        
  }   // if (millis() >= nextControlTime)
    
  // ----- read serial input (BT/console) -------------
  processComm();
  outputConsole();       
  watchdogReset();

  // compute button state (stateButton)
  if (BUTTON_CONTROL){
    if (stopButton.triggered()){
      if (millis() > stateButtonTimeout){
        stateButtonTimeout = millis() + 1000;
        stateButtonTemp++; // next state
        buzzer.sound(SND_READY, true);                                     
      }
                          
    } else {
      if (stateButtonTemp > 0){
        // button released => set stateButton
        stateButtonTimeout = 0;
        stateButton = stateButtonTemp;
        stateButtonTemp = 0;
        CONSOLE.print("stateButton ");
        CONSOLE.println(stateButton);
      }
    }
  }    
}        




// set new robot operation
void setOperation(OperationType op, bool allowRepeat, bool initiatedbyOperator){  
  if ((stateOp == op) && (!allowRepeat)) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.print(op);
  bool error = false;
  bool routingFailed = false;    
  switch (op){
    case OP_IDLE:
      CONSOLE.println(" OP_IDLE");      
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
    case OP_DOCK:
      CONSOLE.println(" OP_DOCK");
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);                
      if ((initiatedbyOperator) || (lastMapRoutingFailed))  maps.clearObstacles();
      if (initiatedbyOperator) {
        dockingInitiatedByOperator = true;            
        dockReasonRainTriggered = false;
      }
      if (maps.startDocking(stateX, stateY)){       
        if (maps.nextPoint(true)) {
          maps.repeatLastMowingPoint();
          lastFixTime = millis();                
          maps.setLastTargetPoint(stateX, stateY);        
          //stateSensor = SENS_NONE;                  
        } else {
          error = true;
          CONSOLE.println("error: no waypoints!");
          //op = stateOp;                
        }
      } else error = true;
      if (error){
        stateSensor = SENS_MAP_NO_ROUTE;
        //op = OP_ERROR;
        routingFailed = true;        
        motor.setMowState(false);
      }
      break;
    case OP_MOW:      
      CONSOLE.println(" OP_MOW");      
      motor.enableTractionMotors(true); // allow traction motors to operate         
      motor.setLinearAngularSpeed(0,0);      
      dockingInitiatedByOperator = false;
      dockReasonRainTriggered = false;
      if ((initiatedbyOperator) || (lastMapRoutingFailed)) maps.clearObstacles();
      if (maps.startMowing(stateX, stateY)){
        if (maps.nextPoint(true)) {
          lastFixTime = millis();                
          maps.setLastTargetPoint(stateX, stateY);        
          //stateSensor = SENS_NONE;
          motor.setMowState(true);                
        } else {
          error = true;
          CONSOLE.println("error: no waypoints!");
          //op = stateOp;                
        }
      } else error = true;
      if (error){
        stateSensor = SENS_MAP_NO_ROUTE;
        //op = OP_ERROR;
        routingFailed = true;
        motor.setMowState(false);
      }
      break;
    case OP_CHARGE:
      CONSOLE.println(" OP_CHARGE");
      //motor.stopImmediately(true); // do not use PID to get to stop 
      motor.setLinearAngularSpeed(0,0, false); 
      motor.setMowState(false);     
      //motor.enableTractionMotors(false); // keep traction motors off (motor drivers tend to generate some incorrect encoder values when stopped while not turning)                 
      break;
    case OP_ERROR:            
      CONSOLE.println(" OP_ERROR"); 
      if (stateOp == OP_CHARGE){
        CONSOLE.println(" - ignoring because we are charging");
        op = stateOp;
      } else {        
        motor.stopImmediately(true); // do not use PID to get to stop
        //motor.setLinearAngularSpeed(0,0);
        motor.setMowState(false);      
      }  
      break;
  }

  if (routingFailed){
    // map routing failed (e.g. due to invalid GPS etc.), try another map routing after some seconds
    lastMapRoutingFailed = true;
    mapRoutingFailedCounter++;    
    if (mapRoutingFailedCounter > 60){
      op = OP_ERROR;  // too many map routing tries after 10 minutes
      retryOperationTime = 0;
    } else {
      retryOperationTime = millis() + 10000; // try another map routing after 10 seconds
      if (mapRoutingFailedCounter == 30){
        // try GPS reboot after 5 minutes
        if (GPS_REBOOT_RECOVERY){
          gps.reboot();  // try to recover from false GPS fix
        }
        retryOperationTime = millis() + 30000; // wait 30 secs after reboot, then try another map routing
      }     
    }
  } else { 
    lastMapRoutingFailed = false;
    mapRoutingFailedCounter = 0;
    retryOperationTime = 0;
  }  
  stateOp = op;  
  saveState();
}
