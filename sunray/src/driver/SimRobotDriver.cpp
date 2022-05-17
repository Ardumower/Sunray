// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SimRobotDriver.h"
#include "../../config.h"



void SimRobotDriver::begin(){
  CONSOLE.println("using robot driver: SimRobotDriver");
}

bool SimRobotDriver::getRobotID(String &id){
  id = "sim";
  return true;
}

bool SimRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = "XX";
  ver = "XX";
  return true;
}


void SimRobotDriver::run(){  
  
}


// ------------------------------------------------------------------------------------

SimMotorDriver::SimMotorDriver(SimRobotDriver &sr): simRobot(sr){
} 

void SimMotorDriver::begin(){
}

void SimMotorDriver::run(){
}

void SimMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
}

void SimMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
}

void SimMotorDriver::resetMotorFaults(){
}

void SimMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  leftCurrent = 0;
  rightCurrent = 0;
  mowCurrent = 0;
}

void SimMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = 0;
  rightTicks = 0;
  mowTicks = 0;
}


// ------------------------------------------------------------------------------------

SimBatteryDriver::SimBatteryDriver(SimRobotDriver &sr) : simRobot(sr){
}

void SimBatteryDriver::begin(){
}

void SimBatteryDriver::run(){
}    

float SimBatteryDriver::getBatteryVoltage(){
  return 30.0;
}

float SimBatteryDriver::getChargeVoltage(){
  return 0;
}
    
float SimBatteryDriver::getChargeCurrent(){
  return 0;
} 

void SimBatteryDriver::enableCharging(bool flag){
}


void SimBatteryDriver::keepPowerOn(bool flag){  
}


// ------------------------------------------------------------------------------------

SimBumperDriver::SimBumperDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimBumperDriver::begin(){
}

void SimBumperDriver::run(){

}

bool SimBumperDriver::obstacle(){
  return false;
}

void SimBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = false;
  rightBumper = false;
}  	  		    


// ------------------------------------------------------------------------------------


SimStopButtonDriver::SimStopButtonDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimStopButtonDriver::begin(){
}

void SimStopButtonDriver::run(){

}

bool SimStopButtonDriver::triggered(){
  return false; 
}

// ------------------------------------------------------------------------------------


SimRainSensorDriver::SimRainSensorDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimRainSensorDriver::begin(){
}

void SimRainSensorDriver::run(){

}

bool SimRainSensorDriver::triggered(){
  return false; 
}

// ------------------------------------------------------------------------------------

SimLiftSensorDriver::SimLiftSensorDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimLiftSensorDriver::begin(){
}

void SimLiftSensorDriver::run(){
}

bool SimLiftSensorDriver::triggered(){
  return false;
}


// ------------------------------------------------------------------------------------

SimBuzzerDriver::SimBuzzerDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimBuzzerDriver::begin(){
}

void SimBuzzerDriver::run(){
}

void SimBuzzerDriver::noTone(){
}

void SimBuzzerDriver::tone(int freq){
}

// ------------------------------------------------------------------------------------


SimImuDriver::SimImuDriver(SimRobotDriver &sr): simRobot(sr){    
}

void SimImuDriver::detect(){  
}
   

bool SimImuDriver::begin(){ 
    CONSOLE.println("using imu driver: SimImuDriver");
    return true;
}


void SimImuDriver::run(){
}


bool SimImuDriver::isDataAvail(){
    roll = 0;
    pitch = 0;
    yaw = 0;    
    return true;
}         
    
void SimImuDriver::resetData(){
}

// -------------------------------------------------------------------------------------


SimGpsDriver::SimGpsDriver(SimRobotDriver &sr): simRobot(sr){
}

void SimGpsDriver::begin(Client &client, char *host, uint16_t port){
}
    
    
void SimGpsDriver::begin(HardwareSerial& bus,uint32_t baud){
}

    
void SimGpsDriver::run(){
}
    
    
bool SimGpsDriver::configure(){  
}
    
    
void SimGpsDriver::reboot(){
}





