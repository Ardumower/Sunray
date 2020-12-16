// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SerialRobotDriver.h"


void SerialRobotComm::begin(){

}

void SerialRobotComm::run(){
}


// ------------------------------------------------------------------------------------

SerialMotorDriver::SerialMotorDriver(){
} 

void SerialMotorDriver::begin(){
}

void SerialMotorDriver::run(){
}

void SerialMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
}

void SerialMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
}

void SerialMotorDriver::resetMotorFaults(){
}

void SerialMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {

}

void SerialMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
}


// ------------------------------------------------------------------------------------

void SerialBatteryDriver::begin(){
}

void SerialBatteryDriver::run(){
}    

float SerialBatteryDriver::getBatteryVoltage(){
}

float SerialBatteryDriver::getChargeVoltage(){
}
    
float SerialBatteryDriver::getChargeCurrent(){
} 

void SerialBatteryDriver::enableCharging(bool flag){
}

void SerialBatteryDriver::keepPowerOn(bool flag){
}


// ------------------------------------------------------------------------------------

void SerialBumperDriver::begin(){
}

void SerialBumperDriver::run(){

}

bool SerialBumperDriver::obstacle(){
}

void SerialBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){

}  	  		    


