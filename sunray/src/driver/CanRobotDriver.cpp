// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "CanRobotDriver.h"
#include "../../config.h"
#include "../../ioboard.h"

//#define COMM  ROBOT

//#define DEBUG_CAN_ROBOT 1

void CanRobotDriver::begin(){
  CONSOLE.println("using robot driver: CanRobotDriver");
  //COMM.begin(ROBOT_BAUDRATE);
  can.begin();
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  encoderTicksMow = 0;
  chargeVoltage = 0;
  chargeCurrent = 0;  
  batteryVoltage = 28;
  cpuTemp = 30;
  mowCurr = 0;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  resetMotorTicks = true;
  batteryTemp = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredRain = false;
  triggeredStopButton = false;
  triggeredLift = false;
  motorFault = false;
  mcuCommunicationLost = true;
  nextSummaryTime = 0;
  nextConsoleTime = 0; 
  nextMotorTime = 0;
  nextTempTime = 0;
  nextWifiTime = 0;
  nextLedTime = 0;
  ledPanelInstalled = true;
  cmdMotorResponseCounter = 0;
  cmdSummaryResponseCounter = 0;
  cmdMotorCounter = 0;
  cmdSummaryCounter = 0;
  consoleCounter = 0;
  requestLeftPwm = requestRightPwm = requestMowPwm = 0;
  robotID = "XX";
  ledStateWifiInactive = false;
  ledStateWifiConnected = false;
  ledStateGpsFix = false;
  ledStateGpsFloat = false;
  ledStateShutdown = false;  
  ledStateError = false;
  ledStateShutdown = false;

  #ifdef __linux__
    CONSOLE.println("reading robot ID...");
    Process p;
    p.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
	  robotID = p.readString();    
    robotID.trim();
    
  #endif
}

bool CanRobotDriver::getRobotID(String &id){
  id = robotID;
  return true;
}

bool CanRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = "OWL";
  ver = "0.0.1";
  return true;
}

float CanRobotDriver::getCpuTemperature(){
  #ifdef __linux__
    return cpuTemp;
  #else
    return -9999;
  #endif
}

void CanRobotDriver::updateCpuTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;        
    while (cpuTempProcess.available()) s+= (char)cpuTempProcess.read();
    if (s.length() > 0) {
      cpuTemp = s.toFloat() / 1000.0;    
      //CONSOLE.print("updateCpuTemperature cpuTemp=");
      //CONSOLE.println(cpuTemp);
    }
    cpuTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");      
    //unsigned long duration = millis() - startTime;        
    //CONSOLE.print("updateCpuTemperature duration: ");
    //CONSOLE.println(duration);        
  #endif
}

void CanRobotDriver::updateWifiConnectionState(){
  #ifdef __linux__
    //unsigned long startTime = millis();   
    String s; 
    while (wifiStatusProcess.available()) s+= (char)wifiStatusProcess.read(); 
    if (s.length() > 0){    
      s.trim();
      //CONSOLE.print("updateWifiConnectionState state=");
      //CONSOLE.println(s);
      // DISCONNECTED, SCANNING, INACTIVE, COMPLETED 
      //CONSOLE.println(s);
      ledStateWifiConnected = (s == "COMPLETED");
      ledStateWifiInactive = (s == "INACTIVE");                   
    }  
    wifiStatusProcess.runShellCommand("wpa_cli -i wlan0 status | grep wpa_state | cut -d '=' -f2");  
    //unsigned long duration = millis() - startTime;        
    //CONSOLE.print("updateWifiConnectionState duration: ");
    //CONSOLE.println(duration);
  #endif
}


// send CAN request 
void CanRobotDriver::sendCanData(int destNodeId, canCmdType_t cmd, canValueType_t val, canDataType_t data){        
    can_frame_t frame;
    frame.can_id = OWL_DRIVE_MSG_ID;    
    if (cmd == can_cmd_request){
      frame.can_dlc = 4;
    } else {
      frame.can_dlc = 8;
    }
    canNodeType_t node;
    node.sourceAndDest.sourceNodeID = MY_NODE_ID;
    node.sourceAndDest.destNodeID = destNodeId;    
    frame.data[0] = node.byteVal[0];
    frame.data[1] = node.byteVal[1];    
    frame.data[2] = cmd;    
    frame.data[3] = val;
    frame.data[4] = data.byteVal[0];
    frame.data[5] = data.byteVal[1];
    frame.data[6] = data.byteVal[2];
    frame.data[7] = data.byteVal[3];
    can.write(frame);
}



// request MCU SW version
void CanRobotDriver::requestVersion(){
}


// request MCU summary
void CanRobotDriver::requestSummary(){
}


// request MCU motor PWM
void CanRobotDriver::requestMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  canDataType_t data;

  data.floatVal = ((float)leftPwm) / 255.0;  
  sendCanData(LEFT_MOTOR_NODE_ID, can_cmd_set, can_val_pwm_speed, data);  
  sendCanData(LEFT_MOTOR_NODE_ID, can_cmd_request, can_val_odo_ticks, data);    
  
  data.floatVal = ((float)rightPwm) / 255.0;    
  sendCanData(RIGHT_MOTOR_NODE_ID, can_cmd_set, can_val_pwm_speed, data);
  sendCanData(RIGHT_MOTOR_NODE_ID, can_cmd_request, can_val_odo_ticks, data);    
  
  data.floatVal = ((float)mowPwm) / 255.0;  
  sendCanData(MOW_MOTOR_NODE_ID, can_cmd_set, can_val_pwm_speed, data);
  sendCanData(MOW_MOTOR_NODE_ID, can_cmd_request, can_val_odo_ticks, data);      

  cmdMotorCounter++;
}

void CanRobotDriver::motorResponse(){
  cmdMotorResponseCounter++;
  mcuCommunicationLost=false;
}


void CanRobotDriver::versionResponse(){
}


void CanRobotDriver::summaryResponse(){
}

// process response
void CanRobotDriver::processResponse(){
  can_frame_t frame;
  while (can.available()){
    if (can.read(frame)){
        //CONSOLE.println("can.read");                
        canNodeType_t node;
        node.byteVal[0] = frame.data[0];
        node.byteVal[1] = frame.data[1];    
      
        int cmd = frame.data[2];     
        canValueType_t val = ((canValueType_t)frame.data[3]);            
        canDataType_t data;
        data.byteVal[0] = frame.data[4];
        data.byteVal[1] = frame.data[5];
        data.byteVal[2] = frame.data[6];
        data.byteVal[3] = frame.data[7];    

        if (cmd == can_cmd_info){
            //CONSOLE.println("can_cmd_info");                
            // info value (volt, velocity, position, ...)
            switch (val){              
              case can_val_odo_ticks:
                switch(node.sourceAndDest.sourceNodeID){
                  case LEFT_MOTOR_NODE_ID:
                    //CONSOLE.println("encoderTicksLeft");
                    encoderTicksLeft = data.ofsAndByte.ofsVal;
                    motorResponse();
                    break;
                  case RIGHT_MOTOR_NODE_ID:
                    encoderTicksRight = data.ofsAndByte.ofsVal;
                    motorResponse();
                    break;
                  case MOW_MOTOR_NODE_ID:
                    encoderTicksMow = data.ofsAndByte.ofsVal;
                    motorResponse();
                    break;
                }                
                break;
                            
            }
        }     
    }
  }
}

void CanRobotDriver::run(){  
  processResponse();
  if (millis() > nextMotorTime){
    nextMotorTime = millis() + 20; // 50 hz
    while (can.available()){
      can_frame_t frame;
      can.read(frame);
    }
    requestMotorPwm(requestLeftPwm, requestRightPwm, requestMowPwm);    
  }
  if (millis() > nextSummaryTime){
    nextSummaryTime = millis() + 500; // 2 hz
    requestSummary();
  }
  if (millis() > nextConsoleTime){
    nextConsoleTime = millis() + 1000;  // 1 hz    
    bool printConsole = false;
    if (consoleCounter == 10){
      printConsole = true;
      consoleCounter = 0;
    }
    if (printConsole){
      CONSOLE.print("CAN: tx=");
      CONSOLE.print(can.frameCounterTx);
      CONSOLE.print(" rx=");
      CONSOLE.print(can.frameCounterRx);    
      CONSOLE.print(" ticks=");
      CONSOLE.print(encoderTicksLeft);
      CONSOLE.print(","); 
      CONSOLE.print(encoderTicksRight);
      CONSOLE.print(" pwm=");
      CONSOLE.print(requestLeftPwm);
      CONSOLE.print(","); 
      CONSOLE.println(requestRightPwm);  
    }

    if (!mcuCommunicationLost){
      if (mcuFirmwareName == ""){
        requestVersion();
      }
    }    
    if ((cmdMotorCounter > 0) && (cmdMotorResponseCounter == 0)){
      CONSOLE.println("WARN: resetting motor ticks");
      resetMotorTicks = true;
      mcuCommunicationLost = true;
    }    
    if ( (cmdMotorResponseCounter < 30) ) { // || (cmdSummaryResponseCounter == 0) ){
      CONSOLE.print("WARN: CanRobot unmet communication frequency: motorFreq=");
      CONSOLE.print(cmdMotorCounter);
      CONSOLE.print("/");
      CONSOLE.print(cmdMotorResponseCounter);
      CONSOLE.print("  summaryFreq=");
      CONSOLE.print(cmdSummaryCounter);
      CONSOLE.print("/");
      CONSOLE.println(cmdSummaryResponseCounter);
      if (cmdMotorResponseCounter == 0){
        // FIXME: maybe reset motor PID controls here?
      }
    }     
    cmdMotorCounter=cmdMotorResponseCounter=cmdSummaryCounter=cmdSummaryResponseCounter=0;    
    consoleCounter++;
  }  
  if (millis() > nextLedTime){
    nextLedTime = millis() + 3000;  // 3 sec
    //updatePanelLEDs();
  }
  if (millis() > nextTempTime){
    nextTempTime = millis() + 59000; // 59 sec
    updateCpuTemperature();
    if (cpuTemp < 60){      
      //setFanPowerState(false);
    } else if (cpuTemp > 65){
      //setFanPowerState(true);
    }
  }
  if (millis() > nextWifiTime){
    nextWifiTime = millis() + 7000; // 7 sec
    updateWifiConnectionState();
  }
}


// ------------------------------------------------------------------------------------

CanMotorDriver::CanMotorDriver(CanRobotDriver &sr): canRobot(sr){
} 

void CanMotorDriver::begin(){
  lastEncoderTicksLeft=0;
  lastEncoderTicksRight=0;
  lastEncoderTicksMow=0;         
}

void CanMotorDriver::run(){
}

void CanMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  //canRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
  canRobot.requestLeftPwm = leftPwm;
  canRobot.requestRightPwm = rightPwm;
  // Alfred mowing motor driver seem to start start mowing motor more successfully with full PWM (100%) values...  
  if (mowPwm > 0) mowPwm = 255;
    else if (mowPwm < 0) mowPwm = -255;
  canRobot.requestMowPwm = mowPwm;
}

void CanMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = canRobot.motorFault;
  rightFault = canRobot.motorFault;
  if (canRobot.motorFault){
    CONSOLE.print("canRobot: motorFault (lefCurr=");
    CONSOLE.print(canRobot.motorLeftCurr);
    CONSOLE.print(" rightCurr=");
    CONSOLE.print(canRobot.motorRightCurr);
    CONSOLE.print(" mowCurr=");
    CONSOLE.println(canRobot.mowCurr);
  }
  mowFault = false;
}

void CanMotorDriver::resetMotorFaults(){
  CONSOLE.println("canRobot: resetting motor fault");
  //canRobot.requestMotorPwm(1, 1, 0);
  //delay(1);
  //canRobot.requestMotorPwm(0, 0, 0);
}

void CanMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent) {  
  //leftCurrent = 0.5;
  //rightCurrent = 0.5;
  //mowCurrent = 0.8;
  leftCurrent = canRobot.motorLeftCurr;
  rightCurrent = canRobot.motorRightCurr;
  mowCurrent = canRobot.mowCurr;
}

void CanMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  if (canRobot.mcuCommunicationLost) {
    //CONSOLE.println("getMotorEncoderTicks: no ticks!");    
    leftTicks = rightTicks = 0; mowTicks = 0;
    return;
  }
  if (canRobot.resetMotorTicks){
    canRobot.resetMotorTicks = false;
    //CONSOLE.println("getMotorEncoderTicks: resetMotorTicks");
    lastEncoderTicksLeft = canRobot.encoderTicksLeft;
    lastEncoderTicksRight = canRobot.encoderTicksRight;
    lastEncoderTicksMow = canRobot.encoderTicksMow;
  }
  leftTicks = canRobot.encoderTicksLeft - lastEncoderTicksLeft;
  rightTicks = canRobot.encoderTicksRight - lastEncoderTicksRight;
  mowTicks = canRobot.encoderTicksMow - lastEncoderTicksMow;
  if (leftTicks > 5000){
    leftTicks = 0;
  }
  if (rightTicks > 5000){
    rightTicks = 0;
  } 
  if (mowTicks > 5000){
    mowTicks = 0;
  }
  lastEncoderTicksLeft = canRobot.encoderTicksLeft;
  lastEncoderTicksRight = canRobot.encoderTicksRight;
  lastEncoderTicksMow = canRobot.encoderTicksMow;
}


// ------------------------------------------------------------------------------------

CanBatteryDriver::CanBatteryDriver(CanRobotDriver &sr) : canRobot(sr){
  mcuBoardPoweredOn = true;
  nextADCTime = 0;
  nextTempTime = 0;
  batteryTemp = 0;
  adcTriggered = false;
  linuxShutdownTime = 0;
}

void CanBatteryDriver::begin(){
}

void CanBatteryDriver::run(){
  if (millis() > nextTempTime){
    nextTempTime = millis() + 57000; // 57 sec
    updateBatteryTemperature();
  }
}    

void CanBatteryDriver::updateBatteryTemperature(){
  #ifdef __linux__
    //unsigned long startTime = millis();
    String s;        
    while (batteryTempProcess.available()) s+= (char)batteryTempProcess.read();
    if (s.length() > 0) {
      batteryTemp = s.toFloat() / 1000.0;    
      //CONSOLE.print("updateBatteryTemperature batteryTemp=");
      //CONSOLE.println(batteryTemp);
    }
    batteryTempProcess.runShellCommand("cat /sys/class/thermal/thermal_zone0/temp");  
    //unsigned long duration = millis() - startTime;        
    //CONSOLE.print("updateBatteryTemperature duration: ");
    //CONSOLE.println(duration);        
  #endif
}


float CanBatteryDriver::getBatteryTemperature(){
  #ifdef __linux__
    return -9999; //batteryTemp; // linux reported bat temp not useful as seem to be constant 31 degree
  #else
    return -9999;
  #endif
}

float CanBatteryDriver::getBatteryVoltage(){
  #ifdef __linux__        
    if (canRobot.mcuCommunicationLost){
      // return 0 volt if MCU PCB is connected and powered-off (Linux will shutdown)
      //if (!mcuBoardPoweredOn) return 0;
      // return 28 volts if MCU PCB is not connected (so Linux can be tested without MCU PCB 
      // and will not shutdown if mower is not connected)      
      return 28;      
    }
  #endif         
  return canRobot.batteryVoltage;
}

float CanBatteryDriver::getChargeVoltage(){
  return canRobot.chargeVoltage;
}
    
float CanBatteryDriver::getChargeCurrent(){
  return canRobot.chargeCurrent;
} 

void CanBatteryDriver::enableCharging(bool flag){
}


void CanBatteryDriver::keepPowerOn(bool flag){
  #ifdef __linux__
    if (flag){
      // keep power on
      linuxShutdownTime = 0;
      canRobot.ledStateShutdown = false;
    } else {
      // shutdown linux - request could be for two reasons:
      // 1. battery voltage sent by MUC-PCB seem to be too low 
      // 2. MCU-PCB is powered-off 
      if (linuxShutdownTime == 0){
        linuxShutdownTime = millis() + 5000; // some timeout 
        // turn off panel LEDs
        canRobot.ledStateShutdown = true;
        //canRobot.updatePanelLEDs();        
      }
      if (millis() > linuxShutdownTime){
        linuxShutdownTime = millis() + 10000; // re-trigger linux command after 10 secs
        CONSOLE.println("LINUX will SHUTDOWN!");
        // switch-off fan via port-expander PCA9555     
        //canRobot.setFanPowerState(false);
        Process p;
        p.runShellCommand("shutdown now");
      }
    }   
  #endif  
}


// ------------------------------------------------------------------------------------

CanBumperDriver::CanBumperDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanBumperDriver::begin(){
}

void CanBumperDriver::run(){

}

bool CanBumperDriver::obstacle(){
  return (canRobot.triggeredLeftBumper || canRobot.triggeredRightBumper); 
}

bool CanBumperDriver::getLeftBumper(){
  return (canRobot.triggeredLeftBumper);
}

bool CanBumperDriver::getRightBumper(){
  return (canRobot.triggeredRightBumper);
}	

void CanBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = canRobot.triggeredLeftBumper;
  rightBumper = canRobot.triggeredRightBumper;
}  	  		    


// ------------------------------------------------------------------------------------


CanStopButtonDriver::CanStopButtonDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanStopButtonDriver::begin(){
}

void CanStopButtonDriver::run(){

}

bool CanStopButtonDriver::triggered(){
  return (canRobot.triggeredStopButton); 
}

// ------------------------------------------------------------------------------------


CanRainSensorDriver::CanRainSensorDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanRainSensorDriver::begin(){
}

void CanRainSensorDriver::run(){

}

bool CanRainSensorDriver::triggered(){
  return (canRobot.triggeredRain); 
}

// ------------------------------------------------------------------------------------

CanLiftSensorDriver::CanLiftSensorDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanLiftSensorDriver::begin(){
}

void CanLiftSensorDriver::run(){
}

bool CanLiftSensorDriver::triggered(){
  return (canRobot.triggeredLift);
}


// ------------------------------------------------------------------------------------

CanBuzzerDriver::CanBuzzerDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanBuzzerDriver::begin(){
}

void CanBuzzerDriver::run(){
}

void CanBuzzerDriver::noTone(){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, false);
}

void CanBuzzerDriver::tone(int freq){
  ioExpanderOut(EX2_I2C_ADDR, EX2_BUZZER_PORT, EX2_BUZZER_PIN, true);
}



