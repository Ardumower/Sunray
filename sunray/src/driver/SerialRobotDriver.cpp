// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SerialRobotDriver.h"
#include "../../config.h"

#define COMM  Serial1



void SerialRobot::begin(){
  COMM.begin(115200);
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  chargeVoltage = 0;  
  batteryVoltage = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
}

void SerialRobot::sendRequest(String req){
  req += F("\r\n");               
  COMM.print(req);
}

void SerialRobot::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  String req;
  req += "AT+M,";
  req += leftPwm;
  req += ",";
  req += rightPwm;
  req += ",";  
  req += mowPwm;
  sendRequest(req);
}

void SerialRobot::motorResponse(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int motorLeftImp=0;
  int motorRightImp=0;
  int motorMowImp =0;
  float chgVoltage=0;
  int bumper=0;
  int lift=0;
  int stopButton=0; 
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //Serial.print("ch=");
    //Serial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      float value = cmd.substring(lastCommaIdx+1, idx+1).toFloat();
      if (counter == 1){                            
        motorLeftImp = value;
      } else if (counter == 2){
        motorRightImp = value;
      } else if (counter == 3){
        motorMowImp = value;
      } else if (counter == 4){
        chgVoltage = value;
      } else if (counter == 5){
        bumper = value;
      } else if (counter == 6){
        lift = value;
      } else if (counter == 7){
        stopButton = value;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }
  encoderTicksLeft = motorLeftImp;
  encoderTicksRight = motorRightImp;
  chargeVoltage = chgVoltage;
  triggeredLeftBumper = (bumper != 0);
}

// process response
void SerialRobot::processResponse(bool checkCrc){
  cmdResponse = "";      
  if (cmd.length() < 4) return;
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
    if (expectedCrc != crc){
      if (checkCrc){
        CONSOLE.print("CRC ERROR");
        CONSOLE.print(crc,HEX);
        CONSOLE.print(",");
        CONSOLE.print(expectedCrc,HEX);
        CONSOLE.println();
        return;  
      }      
    } else {
      // remove CRC
      cmd = cmd.substring(0, idx);
      //CONSOLE.println(cmd);
    }    
  }     
  if (cmd[0] == 'M') motorResponse();
}


// process console input
void SerialRobot::processComm(){
  char ch;      
  if (COMM.available()){
    //battery.resetIdle();  
    while ( COMM.available() ){               
      ch = COMM.read();          
      if ((ch == '\r') || (ch == '\n')) {        
        //CONSOLE.println(cmd);
        processResponse(false);              
        //CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}


void SerialRobot::run(){
  processComm();
}


// ------------------------------------------------------------------------------------

SerialMotorDriver::SerialMotorDriver(SerialRobot &sr): serialRobot(sr){
} 

void SerialMotorDriver::begin(){
}

void SerialMotorDriver::run(){
}

void SerialMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  serialRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
}

void SerialMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
}

void SerialMotorDriver::resetMotorFaults(){
}

void SerialMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {

}

void SerialMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = serialRobot.encoderTicksLeft;
  rightTicks = serialRobot.encoderTicksRight;
  mowTicks = 0;
  serialRobot.encoderTicksLeft = serialRobot.encoderTicksRight = 0; // reset ticks
}


// ------------------------------------------------------------------------------------

SerialBatteryDriver::SerialBatteryDriver(SerialRobot &sr) : serialRobot(sr){
}

void SerialBatteryDriver::begin(){
}

void SerialBatteryDriver::run(){
}    

float SerialBatteryDriver::getBatteryVoltage(){
  return serialRobot.batteryVoltage;
}

float SerialBatteryDriver::getChargeVoltage(){
  return serialRobot.chargeVoltage;
}
    
float SerialBatteryDriver::getChargeCurrent(){
  return 0;
} 

void SerialBatteryDriver::enableCharging(bool flag){
}

void SerialBatteryDriver::keepPowerOn(bool flag){
}


// ------------------------------------------------------------------------------------

SerialBumperDriver::SerialBumperDriver(SerialRobot &sr): serialRobot(sr){
}

void SerialBumperDriver::begin(){
}

void SerialBumperDriver::run(){

}

bool SerialBumperDriver::obstacle(){
}

void SerialBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = serialRobot.triggeredLeftBumper;
  rightBumper = serialRobot.triggeredRightBumper;
}  	  		    


