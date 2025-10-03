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
#include "events.h"


// Refactored: globals are now members of StateEstimator


void StateEstimator::testRelativeLL(){
  // ---- relativeLL calc test (will detect compiler calculation issues) -------- 
  double absolutePosSourceLat = 52.26742967;
  double absolutePosSourceLon = 8.60921633;
  double gpsLon=8.6091091; 
  double gpsLat=52.2674830; 
  float posN = 0;
  float posE = 0;
  relativeLL(absolutePosSourceLat, absolutePosSourceLon, gpsLat, gpsLon, posN, posE);    
  CONSOLE.print("relativeLL calc test: posN=");
  CONSOLE.print(posN,2);
  CONSOLE.print(" posE=");
  CONSOLE.print(posE,2);
  if ( (abs(posN- 5.93) < 0.01) && (abs(posE- -7.30) < 0.01) ) {
    CONSOLE.println(" TEST SUCCEEDED");
  } else {
    CONSOLE.println(" TEST FAILED");
  }
}



void StateEstimator::begin(){
  // run a quick relativeLL self-check after GPS is initialized
  testRelativeLL();
}

// https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide#using-the-mpu-9250-dmp-arduino-library
// start IMU sensor and calibrate
bool StateEstimator::startIMU(bool forceIMU){    
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
  if (!imuDriver.imuFound) {
    Logger.event(EVT_ERROR_IMU_NOT_CONNECTED);
    return false;
  }  
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
      Logger.event(EVT_ERROR_IMU_NOT_CONNECTED);    
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


void StateEstimator::dumpImuTilt(){
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
void StateEstimator::readIMU(){
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
    if (!this->startIMU(true)){ // restart I2C bus
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


void StateEstimator::resetImuTimeout(){
  imuDataTimeout = millis() + 10000;  
}


// compute robot state (x,y,delta)
// uses complementary filter ( https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/ )
// to fusion GPS heading (long-term) and IMU heading (short-term)
// with IMU: heading (stateDelta) is computed by gyro (stateDeltaIMU)
// without IMU: heading (stateDelta) is computed by odometry (deltaOdometry)
void StateEstimator::computeRobotState(){  
  stateLocalizationMode = LOC_GPS;
  bool useGPSposition = true; // use GPS position?
  bool useGPSdelta = true; // correct yaw with gps delta estimation?
  bool useImuAbsoluteYaw = false; // use IMU yaw absolute value?

  // ------- lidar localization --------------------------
  #ifdef GPS_LIDAR
    useGPSdelta = false;
    useImuAbsoluteYaw = true;
  #endif      
  
  // ------- sideways guidance sheets ---------------------
  #ifdef DOCK_GUIDANCE_SHEET // use guidance sheet for docking/undocking?
    if ( maps.isBetweenLastAndNextToLastDockPoint() ){
      stateLocalizationMode = LOC_GUIDANCE_SHEET;
      useGPSposition = false;
      useGPSdelta = false;
      useImuAbsoluteYaw = false;
    }
  #endif

  // ------- vision (april-tag) --------------------------
  #ifdef DOCK_APRIL_TAG  // use april-tag for docking/undocking?
    if (maps.isBetweenLastAndNextToLastDockPoint() ){
      stateLocalizationMode = LOC_APRIL_TAG;
      useGPSposition = false;
      useGPSdelta = false;
      useImuAbsoluteYaw = false;
      if (stateAprilTagFound){  
        float robotX = stateXAprilTag; // robot-in-april-tag-frame (x towards outside tag, y left, z up)
        float robotY = stateYAprilTag;
        float robotDelta = scalePI(stateDeltaAprilTag);    
        /*CONSOLE.print("APRIL TAG found: ");      
        CONSOLE.print(robotX);
        CONSOLE.print(",");
        CONSOLE.print(robotY);
        CONSOLE.print(",");    
        CONSOLE.println(robotDelta/3.1415*180.0);*/        
        float dockX;
        float dockY;
        float dockDelta;
        if (maps.getDockingPos(dockX, dockY, dockDelta)){
          // transform robot-in-april-tag-frame into world frame
          float worldX = dockX + robotX * cos(dockDelta+3.1415) - robotY * sin(dockDelta+3.1415);
          float worldY = dockY + robotX * sin(dockDelta+3.1415) + robotY * cos(dockDelta+3.1415);            
          stateX = worldX;
          stateY = worldY;
          stateDelta = scalePI(robotDelta + dockDelta);
          if (DOCK_FRONT_SIDE) stateDelta = scalePI(stateDelta + 3.1415);
        }
      }
    }
  #endif

  // map dockpoints setup:                                  |===============================
  //               GPS waypoint         GPS waypoint    outside tag     
  //                 x----------------------x---------------------------------------------O (last dockpoint)
  //                                      outside                inside tag visible      inside tag
  //                                      tag visible       |===============================          
  //                                                           
  // localization:              GPS                     LiDAR          
  
  #ifdef DOCK_REFLECTOR_TAG  // use reflector-tag for docking/undocking?
    if (maps.isBetweenLastAndNextToLastDockPoint()) {
      if (maps.shouldDock) {
        stateReflectorTagOutsideFound = false;        
        stateReflectorUndockCompleted = false;
      }
      if ((stateReflectorTagOutsideFound) && (stateXReflectorTag > 0.5)) stateReflectorUndockCompleted = true;
      if (!stateReflectorUndockCompleted) { 
        stateLocalizationMode = LOC_REFLECTOR_TAG;
        useGPSposition = false;
        useGPSdelta = false;
        useImuAbsoluteYaw = false;
        if (stateReflectorTagFound){  
          // don't use stateXReflectorTag as we don't know which tag was detected   
          float robotX = stateXReflectorTag;  // robot-in-reflector-tag-frame (x towards outside tag, y left, z up)      
          if ((stateXReflectorTag > 0) && (stateXReflectorTagLast < 0)){
            if (!maps.shouldDock){
              stateReflectorTagOutsideFound = true;
            } 
          }        
          stateXReflectorTagLast = stateXReflectorTag;
          float robotY = stateYReflectorTag;
          float robotDelta = scalePI(stateDeltaReflectorTag);    
          /*CONSOLE.print("REFLECTOR TAG found: ");      
          CONSOLE.print(robotX);
          CONSOLE.print(",");
          CONSOLE.print(robotY);
          CONSOLE.print(",");    
          CONSOLE.println(robotDelta/3.1415*180.0);*/        
          float dockX;
          float dockY;
          float dockDelta;
          //int dockPointsIdx = maps.dockPoints.numPoints-1; 
          int dockPointsIdx = maps.dockPointsIdx;
          if (maps.getDockingPos(dockX, dockY, dockDelta, dockPointsIdx)){
            // transform robot-in-reflector-tag-frame into world frame
            robotX = 0.2;
            if (!maps.shouldDock) robotX = -0.2;  
            if (robotX < 0) {
              // flip robot at marker
              //robotX *= -1;
              //robotY *= -1;
            }
            float worldX = dockX + robotX * cos(dockDelta+3.1415) - robotY * sin(dockDelta+3.1415);
            float worldY = dockY + robotX * sin(dockDelta+3.1415) + robotY * cos(dockDelta+3.1415);            
            stateX = worldX;
            stateY = worldY;
            stateDelta = scalePI(robotDelta + dockDelta);
            if (DOCK_FRONT_SIDE) stateDelta = scalePI(stateDelta + 3.1415);
          }
        }
      }
    }
  #endif

  // ---------- odometry ticks ---------------------------
  long leftDelta = motor.motorLeftTicks-stateLeftTicks;
  long rightDelta = motor.motorRightTicks-stateRightTicks;  
  stateLeftTicks = motor.motorLeftTicks;
  stateRightTicks = motor.motorRightTicks;    
    
  float distLeft = ((float)leftDelta) / ((float)motor.ticksPerCm);
  float distRight = ((float)rightDelta) / ((float)motor.ticksPerCm);
  if ( (abs(distLeft) > 10.0) ||  (abs(distRight) > 10.0) ) {
    CONSOLE.print("computeRobotState WARN: distOdometry too large - distLeft=");
    CONSOLE.print(distLeft);
    CONSOLE.print(", distRight=");
    CONSOLE.println(distRight);
    distLeft = 0;
    distRight = 0;
  }  
  float distOdometry = (distLeft + distRight) / 2.0;  
  float deltaOdometry = -(distLeft - distRight) / motor.wheelBaseCm;    

  // ---------- GPS relative/absolute position source -----------------
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
          if (useGPSdelta){
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
      }
      lastPosN = posN;
      lastPosE = posE;
      lastPosDelta = stateDelta;
    } 
    
    if (useGPSposition){
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
  } 

  
  /*
  // for testing lidar marker-based docking without GPS  
  #ifdef DOCK_REFLECTOR_TAG
    static bool initializedXYDelta = false;
    if (useGPSposition){
      if (!initializedXYDelta){
        CONSOLE.println("initializedXYDelta");
        stateX = 0;
        stateY = 0;
        stateDelta = 0;      
        initializedXYDelta = true;
      }
    }
  #endif
  */
  

  // odometry
  stateX += distOdometry/100.0 * cos(stateDelta);
  stateY += distOdometry/100.0 * sin(stateDelta);        
  if (stateOp == OP_MOW) statMowDistanceTraveled += distOdometry/100.0;
  
  if ((imuDriver.imuFound) && (maps.useIMU)) {
    // IMU available and should be used by planner        
    if (useImuAbsoluteYaw){
      stateDelta = imuDriver.yaw; 
    } else {
      stateDelta = scalePI(stateDelta + stateDeltaIMU );          
    }     
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


  // invalid position => reset to zero
  if ( (abs(stateX) > 10000) || (abs(stateY) > 10000) ){
    stateX = 0;
    stateY = 0;
  }
}


// StateEstimator instance is defined in robot.cpp
