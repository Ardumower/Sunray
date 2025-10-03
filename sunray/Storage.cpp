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


double Storage::calcStateCRC(){
  return (stateEstimator.stateOp *10 + maps.mowPointsIdx + maps.dockPointsIdx + maps.freePointsIdx + ((byte)maps.wayMode) 
    + sonar.enabled + stateEstimator.fixTimeout + stateEstimator.setSpeed + ((byte)sonar.enabled)
    + ((byte)stateEstimator.absolutePosSource) + stateEstimator.absolutePosSourceLon + stateEstimator.absolutePosSourceLat + motor.pwmMaxMow 
    + ((byte)stateEstimator.finishAndRestart) + ((byte)motor.motorMowForwardSet) + ((byte)battery.docked) + ((byte)stateEstimator.dockAfterFinish)
    + timetable.crc() );
}

void Storage::dumpState(){
  CONSOLE.print("dumpState: ");
  CONSOLE.print(" X=");
  CONSOLE.print(stateEstimator.stateX);
  CONSOLE.print(" Y=");
  CONSOLE.print(stateEstimator.stateY);
  CONSOLE.print(" delta=");
  CONSOLE.print(stateEstimator.stateDelta);
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
  CONSOLE.print(stateEstimator.stateOp);
  CONSOLE.print(" sensor=");
  CONSOLE.print(stateEstimator.stateSensor);
  CONSOLE.print(" sonar.enabled=");
  CONSOLE.print(sonar.enabled);
  CONSOLE.print(" fixTimeout=");
  CONSOLE.print(stateEstimator.fixTimeout);
  CONSOLE.print(" absolutePosSource=");
  CONSOLE.print(stateEstimator.absolutePosSource);
  CONSOLE.print(" lon=");
  CONSOLE.print(stateEstimator.absolutePosSourceLon);
  CONSOLE.print(" lat=");
  CONSOLE.print(stateEstimator.absolutePosSourceLat);
  CONSOLE.print(" pwmMaxMow=");
  CONSOLE.print(motor.pwmMaxMow);
  CONSOLE.print(" finishAndRestart=");
  CONSOLE.print(stateEstimator.finishAndRestart);
  CONSOLE.print(" motorMowForwardSet=");
  CONSOLE.print(motor.motorMowForwardSet);  
  CONSOLE.print(" dockAfterFinish=");
  CONSOLE.println(stateEstimator.dockAfterFinish);
}

void updateStateOpText(){
  switch (stateEstimator.stateOp){
    case OP_IDLE: stateEstimator.stateOpText = "idle"; break;
    case OP_MOW: stateEstimator.stateOpText = "mow"; break;
    case OP_CHARGE: stateEstimator.stateOpText = "charge"; break;
    case OP_ERROR: 
      stateEstimator.stateOpText = "error (";
      switch (stateEstimator.stateSensor){
        case SENS_NONE: stateEstimator.stateOpText += "none)"; break;
        case SENS_BAT_UNDERVOLTAGE: stateEstimator.stateOpText += "unvervoltage)"; break;            
        case SENS_OBSTACLE: stateEstimator.stateOpText += "obstacle)"; break;      
        case SENS_GPS_FIX_TIMEOUT: stateEstimator.stateOpText += "fix timeout)"; break;
        case SENS_IMU_TIMEOUT: stateEstimator.stateOpText += "imu timeout)"; break;
        case SENS_IMU_TILT: stateEstimator.stateOpText += "imu tilt)"; break;
        case SENS_KIDNAPPED: stateEstimator.stateOpText += "kidnapped)"; break;
        case SENS_OVERLOAD: stateEstimator.stateOpText += "overload)"; break;
        case SENS_MOTOR_ERROR: stateEstimator.stateOpText += "motor error)"; break;
        case SENS_GPS_INVALID: stateEstimator.stateOpText += "gps invalid)"; break;
        case SENS_ODOMETRY_ERROR: stateEstimator.stateOpText += "odo error)"; break;
        case SENS_MAP_NO_ROUTE: stateEstimator.stateOpText += "no map route)"; break;
        case SENS_MEM_OVERFLOW: stateEstimator.stateOpText += "mem overflow)"; break;
        case SENS_BUMPER: stateEstimator.stateOpText += "bumper)"; break;
        case SENS_SONAR: stateEstimator.stateOpText += "sonar)"; break;
        case SENS_LIFT: stateEstimator.stateOpText += "lift)"; break;
        case SENS_RAIN: stateEstimator.stateOpText += "rain)"; break;
        case SENS_STOP_BUTTON: stateEstimator.stateOpText += "stop button)"; break;
        default: stateEstimator.stateOpText += "unknown)"; break;
      }
      break;
    case OP_DOCK: stateEstimator.stateOpText = "dock"; break;
    default: stateEstimator.stateOpText = "unknown"; break;
  }
  switch (gps.solution){
    case SOL_INVALID: stateEstimator.gpsSolText = "invalid"; break;
    case SOL_FLOAT: stateEstimator.gpsSolText = "float"; break;
    case SOL_FIXED: stateEstimator.gpsSolText ="fixed"; break;
    default: stateEstimator.gpsSolText = "unknown";      
  }
}


bool Storage::loadState(){
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
  res &= (stateFile.read((uint8_t*)&stateEstimator.stateX, sizeof(stateEstimator.stateX)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.stateY, sizeof(stateEstimator.stateY)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.stateDelta, sizeof(stateEstimator.stateDelta)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.mowPointsIdx, sizeof(maps.mowPointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.dockPointsIdx, sizeof(maps.dockPointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.freePointsIdx, sizeof(maps.freePointsIdx)) != 0);
  res &= (stateFile.read((uint8_t*)&maps.wayMode, sizeof(maps.wayMode)) != 0);
  res &= (stateFile.read((uint8_t*)&savedOp, sizeof(savedOp)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.stateSensor, sizeof(stateEstimator.stateSensor)) != 0);
  res &= (stateFile.read((uint8_t*)&sonar.enabled, sizeof(sonar.enabled)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.fixTimeout, sizeof(stateEstimator.fixTimeout)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.setSpeed, sizeof(stateEstimator.setSpeed)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.absolutePosSource, sizeof(stateEstimator.absolutePosSource)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.absolutePosSourceLon, sizeof(stateEstimator.absolutePosSourceLon)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.absolutePosSourceLat, sizeof(stateEstimator.absolutePosSourceLat)) != 0);
  res &= (stateFile.read((uint8_t*)&motor.pwmMaxMow, sizeof(motor.pwmMaxMow)) != 0);
  res &= (stateFile.read((uint8_t*)&stateEstimator.finishAndRestart, sizeof(stateEstimator.finishAndRestart)) != 0); 
  res &= (stateFile.read((uint8_t*)&motor.motorMowForwardSet, sizeof(motor.motorMowForwardSet)) != 0); 
  res &= (stateFile.read((uint8_t*)&timetable.timetable, sizeof(timetable.timetable)) != 0);
  res &= (stateFile.read((uint8_t*)&battery.docked, sizeof(battery.docked)) != 0);  
  res &= (stateFile.read((uint8_t*)&stateEstimator.dockAfterFinish, sizeof(stateEstimator.dockAfterFinish)) != 0);
  stateFile.close();  
  CONSOLE.println("ok");
  stateCRC = calcStateCRC();
  dumpState();
  timetable.dump();
  if (getResetCause() == RST_WATCHDOG){
    CONSOLE.println("resuming operation due to watchdog trigger");
    stateEstimator.stateOp = savedOp;
    setOperation(stateEstimator.stateOp, true);
  }
#endif
  return true;
}


bool Storage::saveState(){   
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

  res &= (stateFile.write((uint8_t*)&stateEstimator.stateX, sizeof(stateEstimator.stateX)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.stateY, sizeof(stateEstimator.stateY)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.stateDelta, sizeof(stateEstimator.stateDelta)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.mowPointsIdx, sizeof(maps.mowPointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.dockPointsIdx, sizeof(maps.dockPointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.freePointsIdx, sizeof(maps.freePointsIdx)) != 0);
  res &= (stateFile.write((uint8_t*)&maps.wayMode, sizeof(maps.wayMode)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.stateOp, sizeof(stateEstimator.stateOp)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.stateSensor, sizeof(stateEstimator.stateSensor)) != 0);
  res &= (stateFile.write((uint8_t*)&sonar.enabled, sizeof(sonar.enabled)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.fixTimeout, sizeof(stateEstimator.fixTimeout)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.setSpeed, sizeof(stateEstimator.setSpeed)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.absolutePosSource, sizeof(stateEstimator.absolutePosSource)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.absolutePosSourceLon, sizeof(stateEstimator.absolutePosSourceLon)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.absolutePosSourceLat, sizeof(stateEstimator.absolutePosSourceLat)) != 0);
  res &= (stateFile.write((uint8_t*)&motor.pwmMaxMow, sizeof(motor.pwmMaxMow)) != 0);  
  res &= (stateFile.write((uint8_t*)&stateEstimator.finishAndRestart, sizeof(stateEstimator.finishAndRestart)) != 0);  
  res &= (stateFile.write((uint8_t*)&motor.motorMowForwardSet, sizeof(motor.motorMowForwardSet)) != 0);
  res &= (stateFile.write((uint8_t*)&timetable.timetable, sizeof(timetable.timetable)) != 0);  
  res &= (stateFile.write((uint8_t*)&battery.docked, sizeof(battery.docked)) != 0);
  res &= (stateFile.write((uint8_t*)&stateEstimator.dockAfterFinish, sizeof(stateEstimator.dockAfterFinish)) != 0);  
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
