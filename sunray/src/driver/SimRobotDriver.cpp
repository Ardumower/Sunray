// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SimRobotDriver.h"
#include "../../config.h"



void SimRobotDriver::begin(){
  CONSOLE.println("using robot driver: SimRobotDriver");
  simTicksLeft = simTicksRight = 0;  
  simX = simY = simDelta = 0;
  linearSpeed = angularSpeed = 0;
  leftSpeed = rightSpeed = mowSpeed = 0;
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
  lastEncoderTicksLeft = lastEncoderTicksLeft = 0;
  lastSampleTime = 0;
} 

void SimMotorDriver::begin(){
}

void SimMotorDriver::run(){
}


// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2

void SimMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  

  float deltaT = 0;
  if (lastSampleTime != 0){
    deltaT = ((float)(millis() - lastSampleTime)) / 1000.0;
    if (deltaT < 0.2) return;
  } 
  lastSampleTime = millis();

  leftPwm = -leftPwm;

  float maxSpeed = 0.7;  // m/s  (pwm=255)
  simRobot.leftSpeed = ((float)leftPwm) / 255.0 * maxSpeed; 
  simRobot.rightSpeed = ((float)rightPwm) / 255.0 * maxSpeed;
  simRobot.mowSpeed = ((float)mowPwm) / 255.0;

  
  int leftDeltaTicks = simRobot.leftSpeed / (PI * ((float)WHEEL_DIAMETER) / 1000.0) * ((float)TICKS_PER_REVOLUTION) * deltaT;
  int rightDeltaTicks = simRobot.rightSpeed / (PI * ((float)WHEEL_DIAMETER) / 1000.0) * ((float)TICKS_PER_REVOLUTION) * deltaT;
  
  if (leftPwm >= 0)
    simRobot.simTicksLeft += leftDeltaTicks;
  else
    simRobot.simTicksLeft -= leftDeltaTicks;

  if (rightPwm >= 0)
    simRobot.simTicksRight += rightDeltaTicks;
  else 
    simRobot.simTicksRight -= rightDeltaTicks; 

  simRobot.linearSpeed = (simRobot.rightSpeed + simRobot.leftSpeed) / 2.0;
  float wheelBase = ((float)WHEEL_BASE_CM) / 100.0; 
  simRobot.angularSpeed = (simRobot.rightSpeed - simRobot.leftSpeed) / wheelBase;

  simRobot.simX += cos(simRobot.simDelta) * simRobot.linearSpeed * deltaT;
  simRobot.simY += sin(simRobot.simDelta) * simRobot.linearSpeed * deltaT; 
  simRobot.simDelta += simRobot.angularSpeed * deltaT;

  /*CONSOLE.print("simRobot speed ");
  CONSOLE.print(simRobot.leftSpeed);
  CONSOLE.print(",");
  CONSOLE.print(simRobot.rightSpeed);
  CONSOLE.print("  ticks ");
  CONSOLE.print(simRobot.simTicksLeft);
  CONSOLE.print(",");
  CONSOLE.print(simRobot.simTicksRight);
  CONSOLE.print("  pos ");
  CONSOLE.print(simRobot.simX);
  CONSOLE.print(",");
  CONSOLE.println(simRobot.simY); */  
}

void SimMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = rightFault = mowFault = false;
}

void SimMotorDriver::resetMotorFaults(){
}

void SimMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  leftCurrent = abs(simRobot.leftSpeed) / 2.0;
  rightCurrent = abs(simRobot.rightSpeed) / 2.0;
  mowCurrent = abs(simRobot.mowSpeed);
}

void SimMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = simRobot.simTicksLeft - lastEncoderTicksLeft;
  rightTicks = simRobot.simTicksRight - lastEncoderTicksRight;
  if (leftTicks > 1000){
    leftTicks = 0;
  }
  if (rightTicks > 1000){
    rightTicks = 0;
  } 
  lastEncoderTicksLeft = simRobot.simTicksLeft;
  lastEncoderTicksRight = simRobot.simTicksRight;
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
  nextSampleTime = 0;  
}

void SimImuDriver::detect(){  
  imuFound = true;
}
   

bool SimImuDriver::begin(){ 
  CONSOLE.println("using imu driver: SimImuDriver");
  return true;
}


void SimImuDriver::run(){
}


bool SimImuDriver::isDataAvail(){
  if (millis() > nextSampleTime){
    nextSampleTime = millis() + 200; // 5 hz
    roll = 0;
    pitch = 0;
    yaw = simRobot.simDelta;    
    return true;
  } else {
    return false;
  }
}         
    
void SimImuDriver::resetData(){
}

// -------------------------------------------------------------------------------------


SimGpsDriver::SimGpsDriver(SimRobotDriver &sr): simRobot(sr){
  nextSolutionTime = 0;
  solutionAvail = false;
}

void SimGpsDriver::begin(Client &client, char *host, uint16_t port){
}
    
    
void SimGpsDriver::begin(HardwareSerial& bus,uint32_t baud){
}

    
void SimGpsDriver::run(){
  if (true){
    if (millis() > nextSolutionTime){
      nextSolutionTime = millis() + 200; // 5 hz
      relPosE = simRobot.simX;
      relPosN = simRobot.simY;
      relPosD = 100;
      solution = SOL_FIXED;
      lon = relPosE;
      lat =relPosN;
      height = relPosD;
      accuracy = 0.01;
      hAccuracy = accuracy;
      vAccuracy = accuracy;
      dgpsAge = millis();
      groundSpeed = simRobot.linearSpeed;
      solutionAvail = true;
    }
  }
}
    
    
bool SimGpsDriver::configure(){  
}
    
    
void SimGpsDriver::reboot(){
}





