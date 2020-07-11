  // Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "robot.h"
#include "comm.h"
#include "WiFiEsp.h"
#include "SparkFunMPU9250-DMP.h"
#include "SparkFunHTU21D.h"
#include "pinman.h"
#include "ble.h"
#include "motor.h"
#include "battery.h"
#include "ublox.h"
#include "buzzer.h"
#include "map.h"
#include "config.h"
#include "helper.h"
#include "pid.h"
#include "i2c.h"
#include <Arduino.h>

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))


MPU9250_DMP imu;
Motor motor;
Battery battery;
PinManager pinMan;
UBLOX gps(GPS,GPS_BAUDRATE);
BLEConfig bleConfig;
Buzzer buzzer;
Sonar sonar;
Map maps;
HTU21D myHumidity;
PID pidLine(0.2, 0.01, 0); // not used
PID pidAngle(2, 0.1, 0);  // not used

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor
unsigned long controlLoops = 0;
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
float rollChange = 0;
float pitchChange = 0;

UBLOX::SolType lastSolution = UBLOX::SOL_INVALID;    
unsigned long nextStatTime = 0;
unsigned long statIdleDuration = 0; // seconds
unsigned long statChargeDuration = 0; // seconds
unsigned long statMowDuration = 0; // seconds
unsigned long statMowDurationFloat = 0; // seconds
unsigned long statMowDurationFix = 0; // seconds
unsigned long statMowFloatToFixRecoveries = 0; // counter
unsigned long statImuRecoveries = 0; // counter
float statTempMin = 9999; 
float statTempMax = -9999; 
float statMowMaxDgpsAge = 0; // seconds
float statMowDistanceTraveled = 0; // meter


float lastPosN = 0;
float lastPosE = 0;


unsigned long linearMotionStartTime = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextImuTime = 0;
unsigned long nextTempTime = 0;
unsigned long imuDataTimeout = 0;
bool imuFound = false;
float lastIMUYaw = 0; 

bool wifiFound = false;
char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;        // your network password
WiFiEspServer server(80);
WiFiEspClient client = NULL;
int status = WL_IDLE_STATUS;     // the Wifi radio's status

float dockSignal = 0;
bool foundDockSignal = true;
float dockAngularSpeed = 0.1;


// must be defined to override default behavior
void watchdogSetup (void){} 


// get free memory
int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

// reset motion measurement
void resetMotionMeasurement(){
  linearMotionStartTime = millis();  
  //stateGroundSpeed = 1.0;
}


void startWIFI(){
  WIFI.begin(WIFI_BAUDRATE); 
  WIFI.print("AT\r\n");  
  delay(500);
  String res = "";  
  while (WIFI.available()){
    char ch = WIFI.read();    
    res += ch;
  }
  if (res.indexOf("OK") == -1){
    CONSOLE.println("WIFI (ESP8266) not found!");
    return;
  }    
  WiFi.init(&WIFI);  
  if (WiFi.status() == WL_NO_SHIELD) {
    CONSOLE.println("ERROR: WiFi not present");       
  } else {
    wifiFound = true;
    CONSOLE.println("WiFi found!");       
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
    if (ENABLE_UDP) udpSerial.beginUDP();  
    CONSOLE.print("You're connected with SSID=");    
    CONSOLE.print(WiFi.SSID());
    CONSOLE.print(" and IP=");        
    IPAddress ip = WiFi.localIP();    
    CONSOLE.println(ip);   
    if (ENABLE_SERVER){
      server.begin();
    }
  }    
}


// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
void startIMU(bool forceIMU){    
  // detect MPU9250
  uint8_t data = 0;
  int counter = 0;  
  while ((forceIMU) || (counter < 1)){          
     I2CreadFrom(0x69, 0x75, 1, &data, 1); // whoami register
     CONSOLE.print(F("MPU ID=0x"));
     CONSOLE.println(data, HEX);     
     #if defined MPU6050 || defined MPU9150      
       if (data == 0x68) {
         CONSOLE.println("MPU6050/9150 found");
         imuFound = true;
         break;
       }
     #endif
     #if defined MPU9250 
       if (data == 0x73) {
         CONSOLE.println("MPU9255 found");
         imuFound = true;
         break;
       } else if (data == 0x71) {
         CONSOLE.println("MPU9250 found");
         imuFound = true;
         break;
       }
     #endif
     CONSOLE.println(F("MPU6050/9150/9250/9255 not found - Did you connect AD0 to 3.3v and choose it in config.h?"));          
     I2Creset();  
     Wire.begin();    
     #ifdef I2C_SPEED
       Wire.setClock(I2C_SPEED);   
     #endif
     counter++;
  }  
  if (!imuFound) return;  
  while (true){
    if (imu.begin() == INV_SUCCESS) break;
    CONSOLE.print("Unable to communicate with IMU.");
    CONSOLE.print("Check connections, and try again.");
    CONSOLE.println();
    delay(1000);
  }            
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT  // Enable 6-axis quat
               |  DMP_FEATURE_GYRO_CAL // Use gyro calibration
              , 5); // Set DMP FIFO rate to 5 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive    
  imuIsCalibrating = true;   
  nextImuCalibrationSecond = millis() + 1000;
  imuCalibrationSeconds = 0;
}


// read IMU sensor (and restart if required)
// I2C recovery: It can be minutes or hours, then there's an I2C error (probably due an spike on the 
// SCL/SDA lines) and the I2C bus on the pcb1.3 (and the arduino library) hangs and communication is delayed. 
// We check if the communication is significantly (10ms instead of 1ms) delayed, if so we restart the I2C 
// bus (by clocking out any garbage on the I2C bus) and then restarting the IMU module.
// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/using-the-mpu-9250-dmp-arduino-library
void readIMU(){
  if (!imuFound) return;
  // Check for new data in the FIFO  
  unsigned long startTime = millis();
  bool avail = (imu.fifoAvailable() > 0);
  // check time for I2C access : if too long, there's an I2C issue and we need to restart I2C bus...
  unsigned long duration = millis() - startTime;    
  //CONSOLE.print("duration:");
  //CONSOLE.println(duration);  
  if (duration > 10){
    CONSOLE.print("ERROR IMU timeout: ");
    CONSOLE.println(duration);          
    motor.stopImmediately(true);    
    startIMU(true); // restart I2C bus
    statImuRecoveries++;    
    return;
  }      
  if (avail) {    
    //CONSOLE.println("fifoAvailable");
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {      
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles(false);      
      #ifdef ENABLE_TILT_DETECTION
        rollChange += (imu.roll-stateRoll);
        pitchChange += (imu.pitch-statePitch);               
        rollChange = 0.95 * rollChange;
        pitchChange = 0.95 * pitchChange;
        statePitch = imu.pitch;
        stateRoll = imu.roll;        
        //CONSOLE.print(rollChange/PI*180.0);
        //CONSOLE.print(",");
        //CONSOLE.println(pitchChange/PI*180.0);
        if ( (fabs(scalePI(imu.roll)) > 60.0/180.0*PI) || (fabs(scalePI(imu.pitch)) > 100.0/180.0*PI)
             || (fabs(rollChange) > 30.0/180.0*PI) || (fabs(pitchChange) > 60.0/180.0*PI)   )  {
          CONSOLE.println("ERROR IMU tilt");
          CONSOLE.print("imu ypr=");
          CONSOLE.print(imu.yaw/PI*180.0);
          CONSOLE.print(",");
          CONSOLE.print(imu.pitch/PI*180.0);
          CONSOLE.print(",");
          CONSOLE.print(imu.roll/PI*180.0);
          CONSOLE.print(" rollChange=");
          CONSOLE.print(rollChange/PI*180.0);
          CONSOLE.print(" pitchChange=");
          CONSOLE.println(pitchChange/PI*180.0);
          stateSensor = SENS_IMU_TILT;
          setOperation(OP_ERROR);
        }           
      #endif
      imu.yaw = scalePI(imu.yaw);
      lastIMUYaw = scalePI(lastIMUYaw);
      lastIMUYaw = scalePIangles(lastIMUYaw, imu.yaw);
      stateDeltaIMU = scalePI ( distancePI(imu.yaw, lastIMUYaw) );  
      //CONSOLE.print(imu.yaw);
      //CONSOLE.print(",");
      //CONSOLE.print(stateDeltaIMU/PI*180.0);
      //CONSOLE.println();
      lastIMUYaw = imu.yaw;      
      imuDataTimeout = millis() + 10000;
    }     
  }  
  if (millis() > imuDataTimeout){
    // no IMU data within timeout - this should not happen (I2C module error)
    CONSOLE.println("ERROR IMU data timeout");
    stateSensor = SENS_IMU_TIMEOUT;
    setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);        
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
  return (r == 1);
}


// robot start routine
void start(){  
  pinMan.begin();       
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  pinMode(pinDockingReflector, INPUT);
  digitalWrite(pinBatterySwitch, HIGH);       
  buzzer.begin();      
  CONSOLE.begin(CONSOLE_BAUDRATE);  
  Wire.begin();      
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
  CONSOLE.println(VER);          
  battery.begin();      
  
  bleConfig.run();   
  BLE.println(VER);  
    
  motor.begin();
  sonar.begin();
  
  CONSOLE.print("SERIAL_BUFFER_SIZE=");
  CONSOLE.print(SERIAL_BUFFER_SIZE);
  CONSOLE.println(" (increase if you experience GPS checksum errors)");
  CONSOLE.println("-----------------------------------------------------");
  CONSOLE.println("NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:");
  CONSOLE.println("1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom");
  CONSOLE.println("2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h");
  CONSOLE.println("change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024");
  CONSOLE.println("-----------------------------------------------------");
  
  gps.begin();   
  maps.begin();  
  //maps.clipperTest();
  
  myHumidity.begin();    
  
  // initialize ESP module
  startWIFI();  
  
  startIMU(false);        
  
  buzzer.sound(SND_READY);  
  battery.allowSwitchOff(true);  
  watchdogEnable(10000L);   // 10 seconds  
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
        if (gps.solution == UBLOX::SOL_FIXED) statMowDurationFix++;
          else if (gps.solution == UBLOX::SOL_FLOAT) statMowDurationFloat++;   
        if (gps.solution != lastSolution){      
          if ((lastSolution == UBLOX::SOL_FLOAT) && (gps.solution == UBLOX::SOL_FIXED)) statMowFloatToFixRecoveries++;
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
      && ((gps.solution == UBLOX::SOL_FIXED) || (gps.solution == UBLOX::SOL_FLOAT))  )
  {
    gps.solutionAvail = false;        
    stateGroundSpeed = 0.9 * stateGroundSpeed + 0.1 * gps.groundSpeed;    
    //CONSOLE.println(stateGroundSpeed);
    float distGPS = sqrt( sq(posN-lastPosN)+sq(posE-lastPosE) );
    if ((distGPS > 0.3) || (resetLastPos)){
      resetLastPos = false;
      lastPosN = posN;
      lastPosE = posE;
    } else if (distGPS > 0.1) {       
      if ( (fabs(motor.linearSpeedSet) > 0) && (fabs(motor.angularSpeedSet) /PI *180.0 < 45) ) {  
        stateDeltaGPS = scalePI(atan2(posN-lastPosN, posE-lastPosE));    
        if (motor.linearSpeedSet < 0) stateDeltaGPS = scalePI(stateDeltaGPS + PI); // consider if driving reverse
        //stateDeltaGPS = scalePI(2*PI-gps.heading+PI/2);
        float diffDelta = distancePI(stateDelta, stateDeltaGPS);                 
        if (    (gps.solution == UBLOX::SOL_FIXED)
             || ((gps.solution == UBLOX::SOL_FLOAT) && (maps.useGPSfloatForDeltaEstimation)) )
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
    if (gps.solution == UBLOX::SOL_FIXED) {
      // fix
      lastFixTime = millis();
      stateX = posE;
      stateY = posN;        
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
  
  if ((imuFound) && (maps.useIMU)) {
    // IMU available and should be used by planner
    stateDelta = scalePI(stateDelta + stateDeltaIMU );      
  } else {
    // odometry
    stateDelta = scalePI(stateDelta + deltaOdometry);  
  }
  stateDeltaIMU = 0;
}


// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void controlRobotVelocity(){  
  pt_t target = maps.targetPoint;
  pt_t lastTarget = maps.lastTargetPoint;
  float linear = 1.0;  
  float angular = 0;      
  float targetDelta = pointsAngle(stateX, stateY, target.x, target.y);      
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  float diffDelta = distancePI(stateDelta, targetDelta);                         
  float lateralError = distanceLine(stateX, stateY, lastTarget.x, lastTarget.y, target.x, target.y);        
  float targetDist = maps.distanceToTargetPoint(stateX, stateY);
  float lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);  
  if (SMOOTH_CURVES)
    targetReached = (targetDist < 0.2);    
  else 
    targetReached = (targetDist < 0.05);    
  
  
  if ( (motor.motorLeftOverload) || (motor.motorRightOverload) || (motor.motorMowOverload) ){
    linear = 0.1;  
  }
  if (sonar.obstacle()){
    CONSOLE.println("sonar obstacle!");    
    stateSensor = SENS_OBSTACLE;
    setOperation(OP_ERROR);
    buzzer.sound(SND_STUCK, true);        
  }
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 5000){
      CONSOLE.println("overload!");    
      stateSensor = SENS_OVERLOAD;
      setOperation(OP_ERROR);
      buzzer.sound(SND_STUCK, true);        
    }  
  }
  if (ENABLE_FAULT_DETECTION){
    if (motor.motorError){
      motor.motorError = false;
      CONSOLE.println("motor error!");
      stateSensor = SENS_MOTOR_ERROR;
      setOperation(OP_ERROR);
      buzzer.sound(SND_STUCK, true);        
    }  
  }
  
  if (KIDNAP_DETECT){
    if (fabs(lateralError) > 1.0){ // actually, this should not happen (except something strange is going on...)
      CONSOLE.println("kidnapped!");
      stateSensor = SENS_KIDNAPPED;
      setOperation(OP_ERROR);
      buzzer.sound(SND_STUCK, true);        
   }
  }
    
  // allow rotations only near last or next waypoint
  if ((targetDist < 0.5) || (lastTargetDist < 0.5)) {
    if (SMOOTH_CURVES)
      angleToTargetFits = (fabs(diffDelta)/PI*180.0 < 120);          
    else     
      angleToTargetFits = (fabs(diffDelta)/PI*180.0 < 20);   
  } else angleToTargetFits = true;

               
  if (!angleToTargetFits){
    // angular control (if angle to far away, rotate to next waypoint)
    linear = 0;
    angular = 0.5;               
    if ((!rotateLeft) && (!rotateRight)){ // decide for one rotation direction (and keep it)
      if (diffDelta < 0) rotateLeft = true;
        else rotateRight = true;
    }        
    if (rotateLeft) angular *= -1;            
    resetMotionMeasurement();
    if (fabs(diffDelta)/PI*180.0 < 90){
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
    } else if (     ((setSpeed > 0.2) && (maps.distanceToTargetPoint(stateX, stateY) < 0.3) && (!straight))   // approaching
          || ((linearMotionStartTime != 0) && (millis() < linearMotionStartTime + 3000))                      // leaving  
       ) 
    {
      linear = 0.1; // reduce speed when approaching/leaving waypoints          
    } 
    else {
      if (gps.solution == UBLOX::SOL_FLOAT)        
        linear = min(setSpeed, 0.1); // reduce speed for float solution
      else
        linear = setSpeed;         // desired speed
    }      
    //angular = 3.0 * diffDelta + 3.0 * lateralError;       // correct for path errors 
    float k = 0.5;
    if (maps.trackSlow) k = 0.1;
    angular = diffDelta + atan2(k * lateralError, (0.001 + fabs(motor.linearSpeedSet)));       // correct for path errors           
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
  if (fixTimeout != 0){
    if (millis() > lastFixTime + fixTimeout * 1000.0){
      // stop on fix solution timeout (fixme: optionally: turn on place if fix-timeout)
      linear = 0;
      angular = 0;
      stateSensor = SENS_GPS_FIX_TIMEOUT;
      //angular = 0.2;
    } else {
      if (stateSensor == SENS_GPS_FIX_TIMEOUT) stateSensor = SENS_NONE; // clear fix timeout
    }       
  }     
  motor.setLinearAngularSpeed(linear, angular);    
  if ((gps.solution == UBLOX::SOL_FIXED) || (gps.solution == UBLOX::SOL_FLOAT)){        
    if (linear > 0.06) {
      if ((millis() > linearMotionStartTime + 5000) && (stateGroundSpeed < 0.03)){
        // if in linear motion and not enough ground speed => obstacle
        CONSOLE.println("gps obstacle!");
        stateSensor = SENS_OBSTACLE;
        setOperation(OP_ERROR);
        buzzer.sound(SND_STUCK, true);        
      }
    } else {
      resetMotionMeasurement();
    }  
  } else {
    // no gps solution
    if (REQUIRE_VALID_GPS){
      CONSOLE.println("no gps solution!");
      stateSensor = SENS_GPS_INVALID;
      setOperation(OP_ERROR);
      buzzer.sound(SND_STUCK, true);          
    }
  }
   
  if (targetReached){
    bool straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false)){
      // finish        
      if (stateOp == OP_DOCK){
        CONSOLE.println("docking finished!");
        setOperation(OP_IDLE); 
      } else {
        CONSOLE.println("mowing finished!");
        if (!finishAndRestart){             
          setOperation(OP_IDLE); 
          //setOperation(OP_DOCK);             
        }                   
      }
    } else {      
      // next waypoint          
      //if (!straight) angleToTargetFits = false;      
    }
  }  
}


// docking via IR reflector 
// https://www.ebay.de/itm/IR-Hindernis-Vermeidungs-Sensor-fur-Arduino-KY-032-Infrarot-Detektor/182379351623?hash=item2a76a80e47:g:Qb8AAOSwo4pYRr~S
// https://www.ebay.de/itm/Reflektor-rund-84-mm-fur-Reflex-Reflexions-Lichtschranke-Tor-Antrieb-Schiebetor/300933885600?hash=item46110eaea0:g:o4oAAOxy3zNSmMck
void docking(){
  float signal = 0;
  float dockLinearSpeed = 0.05;  
  bool value = digitalRead(pinDockingReflector);    
  if (value == LOW) signal = 1.0;
  dockSignal = 0.9 * dockSignal + 0.1 * signal;
  if (dockSignal > 0.1){  
    // reflection    
    foundDockSignal = true;
  } else if (dockSignal < 0.1) {
    // no reflection
    if (foundDockSignal){
      dockAngularSpeed *= -1; 
      foundDockSignal = false;
    }        
  }
  motor.setLinearAngularSpeed(dockLinearSpeed, dockAngularSpeed);    
  //CONSOLE.println(v);
}


// robot main loop
void run(){  
  buzzer.run();
  battery.run();
  motor.run();
  sonar.run();
  maps.run();  
  
  // temp
  if (millis() > nextTempTime){
    // https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
    nextTempTime = millis() + 60000;
    stateTemp = myHumidity.readTemperature();
    statTempMin = min(statTempMin, stateTemp);
    statTempMax = max(statTempMax, stateTemp);
    stateHumidity = myHumidity.readHumidity();      
    CONSOLE.print("temp=");
    CONSOLE.print(stateTemp,1);
    CONSOLE.print("  humidity=");
    CONSOLE.println(stateHumidity,0);    
  }
  
  // IMU
  if (millis() > nextImuTime){
    nextImuTime = millis() + 150;        
    //imu.resetFifo();    
    if (imuIsCalibrating) {
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
          imu.resetFifo();
          resetMotionMeasurement();
          imuDataTimeout = millis() + 10000;
        }
      }       
    } else {
      readIMU();    
    }
  }
  
  gps.run();
    
  calcStats();  
  
  computeRobotState();  
  
  if (!imuIsCalibrating){     
    if (millis() >= nextControlTime){        
      nextControlTime = millis() + 20; 
      controlLoops++;    
      
      if (battery.chargerConnected() != stateChargerConnected) {    
        stateChargerConnected = battery.chargerConnected(); 
        if (stateChargerConnected){      
          stateChargerConnected = true;
          setOperation(OP_CHARGE);                
        }           
      }     
      if (battery.chargerConnected()){
        if ((stateOp == OP_IDLE) || (stateOp == OP_CHARGE)){
          maps.setIsDocked(true);               
          maps.setRobotStatePosToDockingPos(stateX, stateY, stateDelta);                       
        }
        battery.resetIdle();        
      } else {
        if ((stateOp == OP_IDLE) || (stateOp == OP_CHARGE)){
          maps.setIsDocked(false);
        }
      }
      if ((stateOp == OP_MOW) ||  (stateOp == OP_DOCK)) {      
        if (stateOp == OP_DOCK){
          //docking();
          controlRobotVelocity();       
        } else {
          controlRobotVelocity();       
        }      
        battery.resetIdle();
        if (battery.underVoltage()){
          stateSensor = SENS_BAT_UNDERVOLTAGE;
          setOperation(OP_IDLE);
          //buzzer.sound(SND_OVERCURRENT, true);        
        } 
      }
      else if (stateOp == OP_CHARGE){      
        if (!battery.chargerConnected()){
          setOperation(OP_IDLE);        
        }
      }
    }    
  }
    
  // ----- read serial input (BT/console) -------------
  processConsole();     
  processBLE();     
  processWifi();
  outputConsole();       
  watchdogReset();     
}


// set new robot operation
void setOperation(OperationType op){  
  if (stateOp == op) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.println(op);
  switch (op){
    case OP_IDLE:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
    case OP_DOCK:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);                
      maps.startDocking(stateX, stateY);
      if (maps.nextPoint(true)) {
        resetMotionMeasurement();                
        maps.setLastTargetPoint(stateX, stateY);        
        stateSensor = SENS_NONE;        
        foundDockSignal = true;
      } else {
        CONSOLE.println("error: no waypoints!");
        op = stateOp;                
      }
      break;
    case OP_MOW:      
      motor.setLinearAngularSpeed(0,0);
      maps.startMowing(stateX, stateY);
      if (maps.nextPoint(true)) {
        resetMotionMeasurement();                
        maps.setLastTargetPoint(stateX, stateY);        
        stateSensor = SENS_NONE;
        motor.setMowState(true);                
      } else {
        CONSOLE.println("error: no waypoints!");
        op = stateOp;                
      }
      break;
    case OP_CHARGE:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
    case OP_ERROR:
      motor.setLinearAngularSpeed(0,0);
      motor.setMowState(false);
      break;
  }
  stateOp = op;  
}

