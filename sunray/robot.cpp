// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include <SD.h>
#ifdef __linux__
  #include <WiFi.h>
#endif

#include "robot.h"
#include "StateEstimator.h"
#include "Storage.h"
#include "Stats.h"
#include "LineTracker.h"
#include "comm.h"
#include "src/op/op.h"
#ifdef __linux__
  #include <BridgeClient.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "PubSubClient.h"
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
#include "helper.h"
#include "buzzer.h"
#include "rcmodel.h"
#include "map.h"
#include "config.h"
#include "reset.h"
#include "cpu.h"
#include "i2c.h"
#include "src/test/test.h"
#include "bumper.h"
#include "mqtt.h"

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

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
  SerialBumperDriver bumperDriver(robotDriver);
  SerialStopButtonDriver stopButton(robotDriver);
  SerialRainSensorDriver rainDriver(robotDriver);
  SerialLiftSensorDriver liftDriver(robotDriver);
  SerialBuzzerDriver buzzerDriver(robotDriver);
#elif DRV_SIM_ROBOT
  SimRobotDriver robotDriver;
  SimMotorDriver motorDriver(robotDriver);
  SimBatteryDriver batteryDriver(robotDriver);
  SimBumperDriver bumperDriver(robotDriver);
  SimStopButtonDriver stopButton(robotDriver);
  SimRainSensorDriver rainDriver(robotDriver);
  SimLiftSensorDriver liftDriver(robotDriver);
  SimBuzzerDriver buzzerDriver(robotDriver);
#else
  AmRobotDriver robotDriver;
  AmMotorDriver motorDriver;
  AmBatteryDriver batteryDriver;
  AmBumperDriver bumperDriver;
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
Bumper bumper;
VL53L0X tof(VL53L0X_ADDRESS_DEFAULT);
Map maps;
RCModel rcmodel;
TimeTable timetable;

int stateButton = 0;  
int stateButtonTemp = 0;
unsigned long stateButtonTimeout = 0;

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor

unsigned long controlLoops = 0;
String stateOpText = "";  // current operation as text
String gpsSolText = ""; // current gps solution as text
float stateTemp = 20; // degreeC
//float stateHumidity = 0; // percent
unsigned long stateInMotionLastTime = 0;
bool stateChargerConnected = false;
bool stateInMotionLP = false; // robot is in angular or linear motion? (with motion low-pass filtering)

unsigned long lastFixTime = 0;
int fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
float lastGPSMotionX = 0;
float lastGPSMotionY = 0;
unsigned long nextGPSMotionCheckTime = 0;

bool finishAndRestart = false;

unsigned long nextBadChargingContactCheck = 0;
unsigned long nextToFTime = 0;
unsigned long linearMotionStartTime = 0;
unsigned long angularMotionStartTime = 0;
unsigned long overallMotionTimeout = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextLedTime = 0;
unsigned long nextImuTime = 0;
unsigned long nextTempTime = 0;
unsigned long imuDataTimeout = 0;
unsigned long nextSaveTime = 0;
unsigned long nextTimetableTime = 0;

//##################################################################################
unsigned long loopTime = millis();
int loopTimeNow = 0;
int loopTimeMax = 0;
float loopTimeMean = 0;
int loopTimeMin = 99999;
unsigned long loopTimeTimer = 0;
unsigned long wdResetTimer = millis();
//##################################################################################

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

int motorErrorCounter = 0;


RunningMedian<unsigned int,3> tofMeasurements;


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





void sensorTest(){
  CONSOLE.println("testing sensors for 60 seconds...");
  unsigned long stopTime = millis() + 60000;  
  unsigned long nextMeasureTime = 0;
  while (millis() < stopTime){
    sonar.run();
    bumper.run();
    liftDriver.run();
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
        CONSOLE.print("bumper (left,right,triggered): ");
        CONSOLE.print(((int)bumper.testLeft()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.testRight()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.obstacle()));
        CONSOLE.print("\t");       
      }
	    #ifdef ENABLE_LIFT_DETECTION 
        CONSOLE.print("lift sensor (triggered): ");		
        CONSOLE.print(((int)liftDriver.triggered()));	
        CONSOLE.print("\t");							            
      #endif  
	
      CONSOLE.println();  
      watchdogReset();
      robotDriver.run();   
    }
  }
  CONSOLE.println("end of sensor test - please ignore any IMU/GPS errors");
}


void startWIFI(){
#ifdef __linux__
  WiFi.begin();
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

  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_A4931");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_JYQD
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_JYQD");
  #endif 

  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_A4931");
  #endif     
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD");
  #endif
  
  CONSOLE.print("MOTOR_FAULT_CURRENT: ");
  CONSOLE.println(MOTOR_FAULT_CURRENT);
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
  #ifdef MAX_MOW_PWM
    CONSOLE.print("MAX_MOW_PWM: ");
    CONSOLE.println(MAX_MOW_PWM);
  #endif
  CONSOLE.print("MOW_FAULT_CURRENT: ");
  CONSOLE.println(MOW_FAULT_CURRENT);
  CONSOLE.print("MOW_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOW_OVERLOAD_CURRENT);
  CONSOLE.print("ENABLE_OVERLOAD_DETECTION: ");
  CONSOLE.println(ENABLE_OVERLOAD_DETECTION);
  CONSOLE.print("ENABLE_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_FAULT_DETECTION);
  CONSOLE.print("ENABLE_FAULT_OBSTACLE_AVOIDANCE: ");
  CONSOLE.println(ENABLE_FAULT_OBSTACLE_AVOIDANCE);
  CONSOLE.print("ENABLE_RPM_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_RPM_FAULT_DETECTION);
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
  CONSOLE.print("BUMPER_DEADTIME: ");
  CONSOLE.println(BUMPER_DEADTIME);
  CONSOLE.print("BUMPER_TRIGGER_DELAY: ");
  CONSOLE.println(BUMPER_TRIGGER_DELAY);
  CONSOLE.print("BUMPER_MAX_TRIGGER_TIME: ");
  CONSOLE.println(BUMPER_MAX_TRIGGER_TIME);  
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
  CONSOLE.print("TARGET_REACHED_TOLERANCE: ");
  CONSOLE.println(TARGET_REACHED_TOLERANCE);
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
  CONSOLE.print("USE_TEMP_SENSOR: ");
  CONSOLE.println(USE_TEMP_SENSOR);
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
  
  // give Arduino IDE users some time to open serial console to actually see very first console messages
  #ifndef __linux__
    delay(1500);
  #endif
    
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
    
  // initialize ESP module
  startWIFI();
  #ifdef ENABLE_NTRIP
    ntrip.begin();  
  #endif
  
  watchdogEnable(15000L);   // 15 seconds  
  
  startIMU(false);        
  
  buzzer.sound(SND_READY);  
  battery.resetIdle();        
  loadState();

  #ifdef DRV_SIM_ROBOT
    robotDriver.setSimRobotPosState(stateX, stateY, stateDelta);
    tester.begin();
  #endif
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
  activeOp->onObstacle();
}


// detect sensor malfunction
void detectSensorMalfunction(){  
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if (motor.odometryError){
      CONSOLE.println("odometry error!");    
      activeOp->onOdometryError();
      return;      
    }
  }
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 20000){
      // one motor is taking too much current over a long time (too high gras etc.) and we should stop mowing
      CONSOLE.println("overload!");    
      activeOp->onMotorOverload();
      return;
    }  
  }
  if (ENABLE_FAULT_OBSTACLE_AVOIDANCE){
    // there is a motor error (either unrecoverable fault signal or a malfunction) and we should try an obstacle avoidance
    if (motor.motorError){
      CONSOLE.println("motor error!");
      activeOp->onMotorError();
      return;      
    }  
  }
}

// detect lift 
// returns true, if lift detected, otherwise false
bool detectLift(){  
  #ifdef ENABLE_LIFT_DETECTION
    if (liftDriver.triggered()) {
      return true;            
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

  if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (bumper.obstacle()) ){  
    CONSOLE.println("bumper obstacle!");    
    statMowBumperCounter++;
    triggerObstacle();    
    return true;
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
  activeOp->onObstacleRotation();
}

// stuck rotate detection (e.g. robot cannot due to an obstacle outside of robot rotation point)
// returns true, if stuck detected, otherwise false
bool detectObstacleRotation(){  
  if (!robotShouldRotate()) {
    return false;
  }  
  if (!OBSTACLE_DETECTION_ROTATION) return false; 
  if (millis() > angularMotionStartTime + 15000) { // too long rotation time (timeout), e.g. due to obstacle
    CONSOLE.println("too long rotation time (timeout) for requested rotation => assuming obstacle");
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
        CONSOLE.println("no IMU rotation speed detected for requested rotation => assuming obstacle");    
        triggerObstacleRotation();
        return true;      
      }
    }
    if (diffIMUWheelYawSpeedLP > 10.0/180.0 * PI) {  // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle
      CONSOLE.println("yaw difference between wheels and IMU for requested rotation => assuming obstacle");            
      triggerObstacleRotation();
      return true;            
    }    
  }
  return false;
}




// robot main loop
void run(){  
  #ifdef ENABLE_NTRIP
    ntrip.run();
  #endif
  #ifdef DRV_SIM_ROBOT
    tester.run();
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
  bumper.run();
  
  // state saving
  if (millis() >= nextSaveTime){  
    nextSaveTime = millis() + 5000;
    saveState();
  }
  
  // temp
  if (millis() > nextTempTime){
    nextTempTime = millis() + 60000;    
    float batTemp = batteryDriver.getBatteryTemperature();
    float cpuTemp = robotDriver.getCpuTemperature();    
    CONSOLE.print("batTemp=");
    CONSOLE.print(batTemp,0);
    CONSOLE.print("  cpuTemp=");
    CONSOLE.print(cpuTemp,0);    
    //logCPUHealth();
    CONSOLE.println();    
    if (batTemp < -999){
      stateTemp = cpuTemp;
    } else {
      stateTemp = batTemp;    
    }
    statTempMin = min(statTempMin, stateTemp);
    statTempMax = max(statTempMax, stateTemp);    
  }
  
  // IMU
  if (millis() > nextImuTime){
    nextImuTime = millis() + 150;        
    //imu.resetFifo();    
    if (imuIsCalibrating) {
      activeOp->onImuCalibration();             
    } else {
      readIMU();    
    }
  }

  // LED states
  if (millis() > nextLedTime){
    nextLedTime = millis() + 1000;
    robotDriver.ledStateGpsFloat = (gps.solution == SOL_FLOAT);
    robotDriver.ledStateGpsFix = (gps.solution == SOL_FIXED);
    robotDriver.ledStateError = (stateOp == OP_ERROR);     
  }

  gps.run();

  if (millis() > nextTimetableTime){
    nextTimetableTime = millis() + 30000;
    gps.decodeTOW();
    timetable.setCurrentTime(gps.hour, gps.min, gps.dayOfWeek);
    timetable.run();
  }

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
    
    if (battery.chargerConnected() != stateChargerConnected) {    
      stateChargerConnected = battery.chargerConnected(); 
      if (stateChargerConnected){      
        // charger connected event        
        activeOp->onChargerConnected();                
      } else {
        activeOp->onChargerDisconnected();
      }            
    }
    if (millis() > nextBadChargingContactCheck) {
      if (battery.badChargerContact()){
        nextBadChargingContactCheck = millis() + 60000; // 1 min.
        activeOp->onBadChargingContactDetected();
      }
    } 

    if (battery.underVoltage()){
      activeOp->onBatteryUndervoltage();
    } 
    else {      
      if (USE_TEMP_SENSOR){
        if (stateTemp > DOCK_OVERHEAT_TEMP){
          activeOp->onTempOutOfRangeTriggered();
        } 
        else if (stateTemp < DOCK_TOO_COLD_TEMP){
          activeOp->onTempOutOfRangeTriggered();
        }
      }
      if (RAIN_ENABLE){
        // rain sensor should trigger serveral times to robustly detect rain (robust rain detection)
        // it should not trigger if one rain drop or wet tree leaves touches the sensor  
        if (rainDriver.triggered()){  
          //CONSOLE.print("RAIN TRIGGERED ");
          activeOp->onRainTriggered();                                                                              
        }                           
      }    
      if (battery.shouldGoHome()){
        if (DOCKING_STATION){
           activeOp->onBatteryLowShouldDock();
        }
      }   
       
      if (battery.chargerConnected()){
        if (battery.chargingHasCompleted()){
          activeOp->onChargingCompleted();
        }
      }        
    } 

    //CONSOLE.print("active:");
    //CONSOLE.println(activeOp->name());
    activeOp->checkStop();
    activeOp->run();     
      
    // process button state
    if (stateButton == 5){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_DOCK, false);
    } else if (stateButton == 6){ 
      stateButton = 0; // reset button state        
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_MOW, false);
    } 
    //else if (stateButton > 0){  // stateButton 1 (or unknown button state)        
    else if (stateButton == 1){  // stateButton 1                   
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_IDLE, false);                             
    } else if (stateButton == 9){
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      cmdSwitchOffRobot();
    } else if (stateButton == 12){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
      #ifdef __linux__
        WiFi.startWifiProtectedSetup();
      #endif
    }

    // update operation type      
    stateOp = activeOp->getGoalOperationType();  
            
  }   // if (millis() >= nextControlTime)
    
  // ----- read serial input (BT/console) -------------
  processComm();
  outputConsole();    

  //##############################################################################

  if(millis() > wdResetTimer + 1000){
    watchdogReset();
  }   

  loopTimeNow = millis() - loopTime;
  loopTimeMin = min(loopTimeNow, loopTimeMin); 
  loopTimeMax = max(loopTimeNow, loopTimeMax);
  loopTimeMean = 0.99 * loopTimeMean + 0.01 * loopTimeNow; 
  loopTime = millis();

  if(millis() > loopTimeTimer + 10000){
    if(loopTimeMax > 500){
      CONSOLE.print("WARNING - LoopTime: ");
    }else{
      CONSOLE.print("Info - LoopTime: ");
    }
    CONSOLE.print(loopTimeNow);
    CONSOLE.print(" - ");
    CONSOLE.print(loopTimeMin);
    CONSOLE.print(" - ");
    CONSOLE.print(loopTimeMean);
    CONSOLE.print(" - ");
    CONSOLE.print(loopTimeMax);
    CONSOLE.println("ms");
    loopTimeMin = 99999; 
    loopTimeMax = 0;
    loopTimeTimer = millis();
  }   
  //##############################################################################

  // compute button state (stateButton)
  if (BUTTON_CONTROL){
    if (stopButton.triggered()){
      if (millis() > stateButtonTimeout){
        stateButtonTimeout = millis() + 1000;
        stateButtonTemp++; // next state
        buzzer.sound(SND_READY, true);
        CONSOLE.print("BUTTON ");
        CONSOLE.print(stateButtonTemp);
        CONSOLE.println("s");                                     
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
void setOperation(OperationType op, bool allowRepeat){  
  if ((stateOp == op) && (!allowRepeat)) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.println(op);
  stateOp = op;  
  activeOp->changeOperationTypeByOperator(stateOp);
  saveState();
}
