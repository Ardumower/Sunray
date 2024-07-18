// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "lidar.h"
#include "../../config.h"



LidarGpsDriver::LidarGpsDriver()
{
  iTOW = 0;
  numSV = 0;
  numSVdgps = 0;
  lon = 0;
  lat = 0;
  height = 0;
  relPosN = 0;
  relPosE = 0;
  relPosD = 0;
  heading = 0;
  groundSpeed = 0;
  accuracy = 0;
  hAccuracy = 0;
  vAccuracy = 0;
  solution = SOL_INVALID;
  solutionAvail = false;
  dgpsAge = 0;
  chksumErrorCounter = 0;
  dgpsChecksumErrorCounter = 0;
  dgpsPacketCounter = 0;
  year = 0;
  month = 0;
  day = 0;
  hour = 0;
  mins = 0;
  sec = 0;
  dayOfWeek = 0; 
}

void LidarGpsDriver::begin(){
  CONSOLE.println("using gps driver: LiDAR");    
  
}

void LidarGpsDriver::begin(HardwareSerial& bus,uint32_t baud)
{	
  CONSOLE.println("LiDAR::begin serial");
  begin();
}

/* starts the tcp communication */
void LidarGpsDriver::begin(Client &client, char *host, uint16_t port)
{
  CONSOLE.println("LidarGpsDriver::begin tcp");
  begin();
}

bool LidarGpsDriver::configure(){  
  CONSOLE.println("using LiDAR..."); 
  return true;
}

void LidarGpsDriver::reboot(){
  CONSOLE.println("reboot LiDAR..."); 
}

void LidarGpsDriver::run(){

}

// ---------------------------------------------------------


LidarImuDriver::LidarImuDriver(){    
  dataAvail = false;
  imuFound = false;
}

void LidarImuDriver::detect(){  
}


bool LidarImuDriver::begin(){ 
  return true;
}


void LidarImuDriver::run(){
}


bool LidarImuDriver::isDataAvail(){
 
    bool res = dataAvail;
    dataAvail = false; 
    return res;

}         
    
void LidarImuDriver::resetData(){
    
}


// ---------------------------------------------------------


LidarBumperDriver::LidarBumperDriver(){ 
  triggerBumper = false;
  triggerNearObstacle = false;
}


void LidarBumperDriver::begin(){
  CONSOLE.println("using bumper driver: LiDAR");    
}

void LidarBumperDriver::run(){

}

bool LidarBumperDriver::nearObstacle(){
  return triggerNearObstacle;
}

bool LidarBumperDriver::obstacle(){
  return triggerBumper;
}

bool LidarBumperDriver::getLeftBumper(){
  return triggerBumper;
}

bool LidarBumperDriver::getRightBumper(){
  return triggerBumper;
}	

void LidarBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = triggerBumper;
  rightBumper = triggerBumper;
}  	  		    


