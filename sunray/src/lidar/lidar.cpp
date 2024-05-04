// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "Arduino.h"
#include "lidar.h"
#include "../../config.h"



LiDAR::LiDAR()
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

void LiDAR::begin(){
  CONSOLE.println("using gps driver: LiDAR");    
  
}

void LiDAR::begin(HardwareSerial& bus,uint32_t baud)
{	
  CONSOLE.println("LiDAR::begin serial");
  begin();
}

/* starts the tcp communication */
void LiDAR::begin(Client &client, char *host, uint16_t port)
{
  CONSOLE.println("LiDAR::begin tcp");
  begin();
}

bool LiDAR::configure(){  
  CONSOLE.println("using LiDAR..."); 
  return true;
}

void LiDAR::reboot(){
  CONSOLE.println("reboot LiDAR..."); 
}

void LiDAR::run(){

}


