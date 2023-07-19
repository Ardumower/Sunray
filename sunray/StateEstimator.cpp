// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include "StateEstimator.h"
#include "src/op/op.h"

#include "config.h"
#include "robot.h"
#include "Stats.h"
#include "helper.h"
#include "i2c.h"


float stateX = 0;  // position-east (m)
float stateY = 0;  // position-north (m)
float stateDelta = 0;  // direction (rad)
float stateRoll = 0;
float statePitch = 0;
float stateDeltaGPS = 0;
float stateDeltaIMU = 0;
float stateGroundSpeed = 0; // m/s

unsigned long stateLeftTicks = 0;
unsigned long stateRightTicks = 0;

float lastPosN = 0;
float lastPosE = 0;
float lastPosDelta = 0;

float stateDeltaLast = 0;
float stateDeltaSpeed = 0;
float stateDeltaSpeedLP = 0;
float stateDeltaSpeedIMU = 0;
float stateDeltaSpeedWheels = 0;
float diffIMUWheelYawSpeed = 0;
float diffIMUWheelYawSpeedLP = 0;

bool gpsJump = false;
bool resetLastPos = true;

float lastIMUYaw = 0; 
float lateralError = 0; // lateral error
float rollChange = 0;
float pitchChange = 0;
bool imuIsCalibrating = false;
int imuCalibrationSeconds = 0;
unsigned long nextImuCalibrationSecond = 0;
unsigned long nextDumpTime = 0;


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
       //stateSensor = SENS_IMU_TIMEOUT;
       activeOp->onImuError();
       //setOperation(OP_ERROR);      
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
      //stateSensor = SENS_IMU_TIMEOUT;
      activeOp->onImuError();
      //setOperation(OP_ERROR);      
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


void dumpImuTilt(){
  if (millis() < nextDumpTime) return;
  nextDumpTime = millis() + 10000;
  CONSOLE.print("IMU tilt: ");
  CONSOLE.print("ypr=");
  CONSOLE.print(imuDriver.yaw/PI*180.0);
  CONSOLE.print(",");
  CONSOLE.print(imuDriver.pitch/PI*180.0);
  CONSOLE.print(",");
  CONSOLE.print(imuDriver.roll/PI*180.0);
  CONSOLE.print(" rollChange=");
  CONSOLE.print(rollChange/PI*180.0);
  CONSOLE.print(" pitchChange=");
  CONSOLE.println(pitchChange/PI*180.0);
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
  if (avail) imuDataTimeout = millis() + 10000; // reset IMU data timeout, if IMU data available
  //CONSOLE.print("duration:");
  //CONSOLE.println(duration);  
  if ((duration > 60) || (millis() > imuDataTimeout)) {
    if (millis() > imuDataTimeout){
      CONSOLE.print("ERROR IMU data timeout: ");
      CONSOLE.print(millis()-imuDataTimeout);
      CONSOLE.println(" (check RTC battery if problem persists)");  
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
        dumpImuTilt();
        activeOp->onImuTilt();
        //stateSensor = SENS_IMU_TILT;
        //setOperation(OP_ERROR);
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


void resetImuTimeout(){
  imuDataTimeout = millis() + 10000;  
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
      lastPosDelta = stateDelta;
    } else if (distGPS > 0.1) {       
      float diffLastPosDelta = distancePI(stateDelta, lastPosDelta);                 
      if (fabs(diffLastPosDelta) /PI * 180.0 < 10){  // robot sensors indicate it is not turning
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
      }
      lastPosN = posN;
      lastPosE = posE;
      lastPosDelta = stateDelta;
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


