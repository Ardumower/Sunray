// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "SimRobotDriver.h"
#include "../../config.h"
#include "../../robot.h"
#include "../../helper.h"


void SimRobotDriver::begin(){
  CONSOLE.println("using robot driver: SimRobotDriver");
  simTicksLeft = simTicksRight = 0;  
  linearSpeed = angularSpeed = 0;
  leftSpeed = rightSpeed = mowSpeed = 0;
  simX = 0; 
  simY = 0;
  simDelta = 0;
  simObstacleX = 0;
  simObstacleY = 0;
  simObstacleRadius = 0;
  robotIsBumpingIntoObstacle = false;
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


float SimRobotDriver::getCpuTemperature(){
  return 65.0;
}  

void SimRobotDriver::run(){  
  
}

void SimRobotDriver::setSimRobotPosState(float x, float y, float delta){
  simX = x;
  simY = y;
  simDelta = delta;
}

void SimRobotDriver::setObstacle(float x, float y, float radius){
  simObstacleX = x;
  simObstacleY = y;
  simObstacleRadius = radius;
}

bool SimRobotDriver::pointIsInsideObstacle(float x, float y){
  float dist = distance(simObstacleX, simObstacleY, x, y);  
  return (dist < simObstacleRadius);
}

// ------------------------------------------------------------------------------------

SimMotorDriver::SimMotorDriver(SimRobotDriver &sr): simRobot(sr){
  lastEncoderTicksLeft = lastEncoderTicksLeft = 0;
  lastSampleTime = 0;
  simOdometryError = false;
  simNoMotion = false;
  simNoRobotYawRotation = false;
  simMotorLeftFault = simMotorRightFault = simMotorMowFault = false;
  simMotorLeftOverload = simMotorRightOverload = simMotorMowOverload = false; 
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

  // undo swapping motor directions (simulator does not like if swapping was enabled in config)
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    leftPwm = -leftPwm;
  #endif

  #ifdef MOTOR_RIGHT_SWAP_DIRECTION 
    rightPwm = -rightPwm;
  #endif

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

  // if simulating no motion: tires turn but robot does not move 
  if (!simNoMotion){
    float x = simRobot.simX + cos(simRobot.simDelta) * simRobot.linearSpeed * deltaT;
    float y = simRobot.simY + sin(simRobot.simDelta) * simRobot.linearSpeed * deltaT;     
    // robot cannot move inside simulated obstacle
    if (simRobot.pointIsInsideObstacle(x, y)){
      //CONSOLE.println("SIM: robotIsBumpingIntoObstacle");      
      simRobot.robotIsBumpingIntoObstacle = true;
    } else {
      simRobot.simX = x;
      simRobot.simY = y;     
      simRobot.robotIsBumpingIntoObstacle = false;
    } 
    // if simulating no yaw rotation: tires turn but robot does not rotate
    if (!simNoRobotYawRotation) simRobot.simDelta += simRobot.angularSpeed * deltaT;
  }

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
  leftFault = simMotorLeftFault;
  rightFault = simMotorRightFault;
  mowFault = simMotorMowFault;  
}

void SimMotorDriver::resetMotorFaults(){
  CONSOLE.println("SimMotorDriver::resetMotorFaults");
  simMotorLeftFault = simMotorRightFault = simMotorMowFault = false;
}

void SimMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  leftCurrent = abs(simRobot.leftSpeed) / 2.0;
  rightCurrent = abs(simRobot.rightSpeed) / 2.0;
  mowCurrent = abs(simRobot.mowSpeed);
  // if overload, motor turns, but takes way more current
  if (simMotorLeftOverload) leftCurrent = 8.0;
  if (simMotorRightOverload) rightCurrent = 8.0;  
  if (simMotorMowOverload) mowCurrent = 8.0;
  // if fault, motor does not turn
  if (simMotorLeftFault) leftCurrent = 0;
  if (simMotorRightFault) rightCurrent = 0;
  if (simMotorMowFault) mowCurrent = 0;
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
  mowTicks = 500;
  if (simOdometryError) leftTicks = rightTicks = mowTicks = 0;
  // if fault, motor does not turn
  if (simMotorLeftFault) leftTicks = 0;
  if (simMotorRightFault) rightTicks = 0;
  if (simMotorMowFault) mowTicks = 0;  
}


void SimMotorDriver::setSimOdometryError(bool flag){
  simOdometryError = flag;
}

void SimMotorDriver::setSimMotorFault(bool leftFlag, bool rightFlag, bool mowFlag){
  simMotorLeftFault = leftFlag;
  simMotorRightFault = rightFlag;
  simMotorMowFault = mowFlag;
}

void SimMotorDriver::setSimMotorOverload(bool leftFlag, bool rightFlag, bool mowFlag){
  simMotorLeftOverload = leftFlag;
  simMotorRightOverload = rightFlag;
  simMotorMowOverload = mowFlag;
}


void SimMotorDriver::setSimNoMotion(bool flag){
  simNoMotion = flag;  
}

void SimMotorDriver::setSimNoRobotYawRotation(bool flag){
  simNoRobotYawRotation = flag;
}
  

// ------------------------------------------------------------------------------------

SimBatteryDriver::SimBatteryDriver(SimRobotDriver &sr) : simRobot(sr){
  simVoltage = 27.0;
  simChargerConnected = false;
  robotIsAtDockingPoint = false;
}

void SimBatteryDriver::begin(){
}

void SimBatteryDriver::run(){
// docking point reached => connect charger  
  float dockX = 0;
  float dockY = 0;
  float dockDelta = 0;  
  if (maps.getDockingPos(dockX, dockY, dockDelta)){
    float dist = distance(simRobot.simX, simRobot.simY, dockX, dockY);  
    robotIsAtDockingPoint = (dist < 0.5);
  } else {
    robotIsAtDockingPoint = false;
  }

  if ((robotIsAtDockingPoint) || (simChargerConnected)){
    // quickly charge robot :-)
    setSimFullyChargedVoltage(true);    
  }
}    

float SimBatteryDriver::getBatteryVoltage(){
  return simVoltage;
}

float SimBatteryDriver::getChargeVoltage(){
  if ((robotIsAtDockingPoint) || (simChargerConnected)) return 30.0;
    else return 4.0;
}
    
float SimBatteryDriver::getChargeCurrent(){
  if (simChargerConnected) return 1.0;
    else return 0.15;
} 

float SimBatteryDriver::getBatteryTemperature(){
  return 50.0;
} 

void SimBatteryDriver::enableCharging(bool flag){
}


void SimBatteryDriver::keepPowerOn(bool flag){  
}


void SimBatteryDriver::setSimUndervoltage(bool flag){
  if (flag) simVoltage = 17.0;
    else simVoltage = 27.0;
}


void SimBatteryDriver::setSimGoDockVoltage(bool flag){
  if (flag) simVoltage = 21.0;
    else simVoltage = 27.0;
}

void SimBatteryDriver::setSimFullyChargedVoltage(bool flag){
  if (flag) simVoltage = 31.5;
    else simVoltage = 27.0;
}

void SimBatteryDriver::setSimChargerConnected(bool flag){
  simChargerConnected = flag;
}

// ------------------------------------------------------------------------------------

SimBumperDriver::SimBumperDriver(SimRobotDriver &sr): simRobot(sr){
  simTriggered = false;
}

void SimBumperDriver::begin(){
}

void SimBumperDriver::run(){

}

bool SimBumperDriver::obstacle(){
  return (simTriggered || simRobot.robotIsBumpingIntoObstacle);
}

bool SimBumperDriver::getLeftBumper(){
  return (simTriggered || simRobot.robotIsBumpingIntoObstacle);
}

bool SimBumperDriver::getRightBumper(){
  return (simTriggered || simRobot.robotIsBumpingIntoObstacle);
}

void SimBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = (simTriggered || simRobot.robotIsBumpingIntoObstacle);
  rightBumper = (simTriggered || simRobot.robotIsBumpingIntoObstacle);
}  	  		

void SimBumperDriver::setSimTriggered(bool flag){
  simTriggered = flag;
}

// ------------------------------------------------------------------------------------


SimStopButtonDriver::SimStopButtonDriver(SimRobotDriver &sr): simRobot(sr){
  simTriggered = false;
}

void SimStopButtonDriver::begin(){
}

void SimStopButtonDriver::run(){

}

bool SimStopButtonDriver::triggered(){
  return simTriggered; 
}

void SimStopButtonDriver::setSimTriggered(bool flag){
  simTriggered = flag;
}
// ------------------------------------------------------------------------------------


SimRainSensorDriver::SimRainSensorDriver(SimRobotDriver &sr): simRobot(sr){
  simTriggered = false;
}

void SimRainSensorDriver::begin(){
}

void SimRainSensorDriver::run(){

}

bool SimRainSensorDriver::triggered(){
  return simTriggered; 
}

void SimRainSensorDriver::setSimTriggered(bool flag){
  simTriggered = flag;
}

// ------------------------------------------------------------------------------------

SimLiftSensorDriver::SimLiftSensorDriver(SimRobotDriver &sr): simRobot(sr){
  simTriggered = false;
}

void SimLiftSensorDriver::begin(){
}

void SimLiftSensorDriver::run(){
}

bool SimLiftSensorDriver::triggered(){
  return simTriggered;
}

void SimLiftSensorDriver::setSimTriggered(bool flag){
  simTriggered = flag;
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
  simNoData = false;
  simDataTimeout = false;
  simTilt = false;
}

void SimImuDriver::detect(){  
  imuFound = true;
}
   

bool SimImuDriver::begin(){ 
  CONSOLE.println("using imu driver: SimImuDriver");
  simNoData = false;
  simDataTimeout = false;
  simTilt = false;
  return true;
}


void SimImuDriver::run(){
}


bool SimImuDriver::isDataAvail(){
  if (simNoData) return false;
  if (simDataTimeout) {
    delay(100);    
  }
  if (millis() > nextSampleTime){
    nextSampleTime = millis() + 200; // 5 hz
    roll = 0;
    pitch = 0;
    if (simTilt) pitch = PI/180.0 * 90;
    yaw = simRobot.simDelta;        
    return true;
  } else {
    return false;
  }
}         
    
void SimImuDriver::resetData(){  
}

void SimImuDriver::setSimDataTimeout(bool flag){
  simDataTimeout = flag;
}

void SimImuDriver::setSimNoData(bool flag){
  simNoData = flag;
}

void SimImuDriver::setSimTilt(bool flag){
  simTilt = flag;  
}

// -------------------------------------------------------------------------------------


SimGpsDriver::SimGpsDriver(SimRobotDriver &sr): simRobot(sr){
  nextSolutionTime = 0;
  solutionAvail = false;
  simGpsJump = false;
  setSimSolution(SOL_FIXED);
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
      if (simGpsJump){
        relPosE = simRobot.simX + 3.0;
        relPosN = simRobot.simY + 3.0; 
      }      
      //solution = SOL_FIXED;
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
  return true;
}
    
    
void SimGpsDriver::reboot(){
  CONSOLE.println("SimGpsDriver::reboot");
}


void SimGpsDriver::setSimSolution(SolType sol){
  solution = sol;  
}


void SimGpsDriver::setSimGpsJump(bool flag){
  simGpsJump = flag;
}


