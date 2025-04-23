// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "Storage.h"
#include "StateEstimator.h"
#include "robot.h"
#include "map.h"
#include "config.h"
#include "reset.h"
#include <Arduino.h>


File stateFile;
double stateCRC = 0;


double calcStateCRC(){
 return (stateOp *10 + maps.mowPointsIdx + maps.dockPointsIdx + maps.freePointsIdx + ((byte)maps.wayMode) 
   + sonar.enabled + fixTimeout + setSpeed + ((byte)sonar.enabled)
   + ((byte)absolutePosSource) + absolutePosSourceLon + absolutePosSourceLat + motor.pwmMaxMow 
   + ((byte)finishAndRestart) + ((byte)motor.motorMowForwardSet) + ((byte)battery.docked) + ((byte)dockAfterFinish)
   + timetable.crc() );
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
  CONSOLE.print(absolutePosSourceLat);
  CONSOLE.print(" pwmMaxMow=");
  CONSOLE.print(motor.pwmMaxMow);
  CONSOLE.print(" finishAndRestart=");
  CONSOLE.print(finishAndRestart);
  CONSOLE.print(" motorMowForwardSet=");
  CONSOLE.print(motor.motorMowForwardSet);  
  CONSOLE.print(" dockAfterFinish=");
  CONSOLE.println(dockAfterFinish);
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
  if (marker != 0x10001007){
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
  res &= (stateFile.read((uint8_t*)&motor.pwmMaxMow, sizeof(motor.pwmMaxMow)) != 0);
  res &= (stateFile.read((uint8_t*)&finishAndRestart, sizeof(finishAndRestart)) != 0); 
  res &= (stateFile.read((uint8_t*)&motor.motorMowForwardSet, sizeof(motor.motorMowForwardSet)) != 0); 
  res &= (stateFile.read((uint8_t*)&timetable.timetable, sizeof(timetable.timetable)) != 0);
  res &= (stateFile.read((uint8_t*)&battery.docked, sizeof(battery.docked)) != 0);  
  res &= (stateFile.read((uint8_t*)&dockAfterFinish, sizeof(dockAfterFinish)) != 0);
  stateFile.close();  
  CONSOLE.println("ok");
  stateCRC = calcStateCRC();
  dumpState();
  timetable.dump();
  if (getResetCause() == RST_WATCHDOG){
    CONSOLE.println("resuming operation due to watchdog trigger");
    stateOp = savedOp;
    setOperation(stateOp, true);
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
  uint32_t marker = 0x10001007;
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
  res &= (stateFile.write((uint8_t*)&motor.pwmMaxMow, sizeof(motor.pwmMaxMow)) != 0);  
  res &= (stateFile.write((uint8_t*)&finishAndRestart, sizeof(finishAndRestart)) != 0);  
  res &= (stateFile.write((uint8_t*)&motor.motorMowForwardSet, sizeof(motor.motorMowForwardSet)) != 0);
  res &= (stateFile.write((uint8_t*)&timetable.timetable, sizeof(timetable.timetable)) != 0);  
  res &= (stateFile.write((uint8_t*)&battery.docked, sizeof(battery.docked)) != 0);
  res &= (stateFile.write((uint8_t*)&dockAfterFinish, sizeof(dockAfterFinish)) != 0);  
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



