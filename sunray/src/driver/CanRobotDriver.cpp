// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "CanRobotDriver.h"
#include "../../config.h"
#include "../../ioboard.h"
#include "../../config.h"
#include "../../robot.h"
#include "../../events.h"

//#define COMM  ROBOT

//#define DEBUG_CAN_ROBOT 1

int MOW_MOTOR_NODE_IDS[] = { MOW1_MOTOR_NODE_ID, MOW2_MOTOR_NODE_ID, MOW3_MOTOR_NODE_ID, MOW4_MOTOR_NODE_ID, MOW5_MOTOR_NODE_ID  };


void CanRobotDriver::begin(){
  CONSOLE.println("using robot driver: CanRobotDriver");
  //COMM.begin(ROBOT_BAUDRATE);
  can.begin();
  encoderTicksLeft = 0;
  encoderTicksRight = 0;
  for (int i=0; i < MOW_MOTOR_COUNT; i++) {
    encoderTicksMow[i] = 0;
    mowCurr[i] = 0;
  }
  chargeVoltage = 0;
  chargeCurrent = 0;  
  batteryVoltage = 28;
  cpuTemp = 30;
  motorLeftCurr = 0;
  motorRightCurr = 0;
  resetMotorTicks = true;
  batteryTemp = 0;
  triggeredLeftBumper = false;
  triggeredRightBumper = false;
  triggeredRain = false;
  triggeredStopButton = false;
  triggeredPushboxStopButton = false;
  triggeredLift = false;
  for (int i=0; i < MOW_MOTOR_COUNT; i++) mowFault[i] = false;
  leftMotorFault = false;
  rightMotorFault = false;
  mcuCommunicationLost = true;
  nextSummaryTime = 0;
  nextCheckErrorTime = 0;
  nextConsoleTime = 0;
  nextMowTime = 0; 
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
  requestReleaseBrakesWhenZero = false;
  requestMowHeightMillimeter = 50;
  motorHeightAngleEndswitch = 0;
  motorHeightAngleEndswitchSet = false;
  motorHeightAngleCurr = 0;
  motorHeightFoundEndswitch = false;
  robotID = "XX";
  ledStateWifiInactive = false;
  ledStateWifiConnected = false;
  ledStateGpsFix = false;
  ledStateGpsFloat = false;
  ledStateShutdown = false;  
  ledStateError = false;
  ledStateShutdown = false;

  #ifdef __linux__
    Process p;
    p.runShellCommand("pwd");
	  String workingDir = p.readString();    
    CONSOLE.print("linux working dir (pwd): ");
    CONSOLE.println(workingDir);

    CONSOLE.println("reading robot ID...");
    Process p2;
    p2.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
	  robotID = p2.readString();    
    robotID.trim();
    
  #endif
  CONSOLE.print("testing unsigned overflow substraction: ");  
  //unsigned short lastV = 65534;
  //unsigned short currV = 1;
  //unsigned short diffV = currV - lastV;
  unsigned long lastV = 65534;
  unsigned long currV = 1;
  unsigned long diffV = (unsigned short) (currV - lastV);  
  CONSOLE.println(diffV);
  //exit(0);
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
void CanRobotDriver::sendCanData(int msgId, int destNodeId, canCmdType_t cmd, int val, canDataType_t data){        
    can_frame_t frame;
    frame.can_id = msgId;    
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
  canDataType_t data;
  data.floatVal = 0;
  
  switch (cmdSummaryCounter % 7){
    case 0:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_stop_button_state, data );  
      break;
    case 1:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_bumper_state, data );
      break;
    case 2:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_battery_voltage, data );
      break;
    case 3:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_rain_state, data );
      break;
    case 4:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_charger_voltage, data );
      break;
    case 5:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_lift_state, data );
      break;
    case 6:
      sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_slow_down_state, data );
      break;
  }
  cmdSummaryCounter++;
}


// request MCU motor PWM
void CanRobotDriver::requestMotorDrivePwm(int leftPwm, int rightPwm, bool requestReleaseBrakesWhenZero){
  canDataType_t data;

  if ((leftPwm == 0) && (requestReleaseBrakesWhenZero)){
    data.byteVal[0] = 0;      
    sendCanData(OWL_DRIVE_MSG_ID, LEFT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_motor_enable, data);
  } else {
    data.floatVal = ((float)leftPwm) / 255.0;  
    sendCanData(OWL_DRIVE_MSG_ID, LEFT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_pwm_speed, data);  
  }
  sendCanData(OWL_DRIVE_MSG_ID, LEFT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_odo_ticks, data);    
  
  if ((rightPwm == 0) && (requestReleaseBrakesWhenZero)){
    data.byteVal[0] = 0;      
    sendCanData(OWL_DRIVE_MSG_ID, RIGHT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_motor_enable, data);  
  } else {
    data.floatVal = ((float)rightPwm) / 255.0;    
    sendCanData(OWL_DRIVE_MSG_ID, RIGHT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_pwm_speed, data);
  }
  sendCanData(OWL_DRIVE_MSG_ID, RIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_odo_ticks, data);    
  cmdMotorCounter++;
}


void CanRobotDriver::requestMotorMowPwm(int mowPwm){
  canDataType_t data;

  #ifdef MAX_MOW_RPM
    // cutter speed (velocity control)
    data.floatVal = ((float)mowPwm) / 255.0 * ((float)MAX_MOW_RPM)/60.0 * 3.1415*2.0;   // convert 0..255 to target velocity (motor radiant/sec)    
    for (int i=0; i < MOW_MOTOR_COUNT; i++){
      sendCanData(OWL_DRIVE_MSG_ID, MOW_MOTOR_NODE_IDS[i], can_cmd_set, owldrv::can_val_velocity, data);
    }
  #else
    // cutter speed (voltage control)
    data.floatVal = ((float)mowPwm) / 255.0;
    for (int i=0; i < MOW_MOTOR_COUNT; i++){    
      sendCanData(OWL_DRIVE_MSG_ID, MOW_MOTOR_NODE_IDS[i], can_cmd_set, owldrv::can_val_pwm_speed, data);
    }
  #endif

  for (int i=0; i < MOW_MOTOR_COUNT; i++){
    sendCanData(OWL_DRIVE_MSG_ID, MOW_MOTOR_NODE_IDS[i], can_cmd_request, owldrv::can_val_odo_ticks, data);
  }  
}


void CanRobotDriver::motorResponse(){
  cmdMotorResponseCounter++;
  mcuCommunicationLost=false;
}

void CanRobotDriver::requestMowHeight(int mowHeightMillimeter){
  //can node 8      angle 6450=60mm   angle -5000=20mm
  float heightEndSwitchMillimeter = 55;  // height at endswitch  (60mm)
  float heightMin = 20;  // min. allowed height   (20mm)
  float heightMax = 90;  // max. allowed height   (60mm)
  float motorAnglePerMillimeter = 325;  // motor angles per millimeter (320)  
  mowHeightMillimeter = max(heightMin, min(mowHeightMillimeter, heightMax));  // limit to allowed min/max  
  canDataType_t data;  
  if (motorHeightFoundEndswitch){
    bool sendTarget = true;
    if (!motorHeightAngleEndswitchSet) sendTarget = false;          
    // convert millimeter to motor angle radiant    
    data.floatVal = motorHeightAngleEndswitch - (((float)(heightEndSwitchMillimeter-mowHeightMillimeter)) * motorAnglePerMillimeter);  
    float diff = abs(motorHeightAngleCurr - data.floatVal);    
    if (diff < 400){
      data.byteVal[0] = 0;
      sendCanData(OWL_DRIVE_MSG_ID, MOW_HEIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_motor_enable, data);        
      sendTarget = false;
    }     
    if (sendTarget) {
      CONSOLE.print("motorHeightAngleCurr=");
      CONSOLE.print(motorHeightAngleCurr);
      CONSOLE.print(",");
      CONSOLE.print(" target=");
      CONSOLE.print(data.floatVal);
      CONSOLE.print(",");
      CONSOLE.print(" diff=");      
      CONSOLE.println(diff);
    }
    //CONSOLE.print("endswitch found - requestMowHeight: ");
    //CONSOLE.print(data.floatVal);
    //CONSOLE.print("(");
    //CONSOLE.print(mowHeightMillimeter);
    //CONSOLE.println("mm)");    
    if (sendTarget) sendCanData(OWL_DRIVE_MSG_ID, MOW_HEIGHT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_target, data);    
  } else {
    if (!motorHeightAngleEndswitchSet){    
      CONSOLE.println("finding endswitch");
      data.floatVal = 10000 * motorAnglePerMillimeter;   // unreachable target (10mm) (find endswitch)   
      //CONSOLE.print("no endswitch found - requestMowHeight: ");
      //CONSOLE.println(data.floatVal);
      sendCanData(OWL_DRIVE_MSG_ID, MOW_HEIGHT_MOTOR_NODE_ID, can_cmd_set, owldrv::can_val_target, data);    
      sendCanData(OWL_DRIVE_MSG_ID, MOW_HEIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_endswitch, data);  
    }
  }
  sendCanData(OWL_DRIVE_MSG_ID, MOW_HEIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_angle, data);  
}

void CanRobotDriver::requestMotorErrorStatus(){
  canDataType_t data;    
  for (int i=0; i < MOW_MOTOR_COUNT; i++){
    sendCanData(OWL_DRIVE_MSG_ID, MOW_MOTOR_NODE_IDS[i], can_cmd_request, owldrv::can_val_error, data);
  }
  sendCanData(OWL_DRIVE_MSG_ID, LEFT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_error, data);
  sendCanData(OWL_DRIVE_MSG_ID, RIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_error, data);
}

void CanRobotDriver::requestMotorMowCurrent(){
  canDataType_t data;
  for (int i=0; i < MOW_MOTOR_COUNT; i++){
    sendCanData(OWL_DRIVE_MSG_ID, MOW_MOTOR_NODE_IDS[i], can_cmd_request, owldrv::can_val_total_current, data);
  }  
  sendCanData(OWL_DRIVE_MSG_ID, LEFT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_total_current, data);
  sendCanData(OWL_DRIVE_MSG_ID, RIGHT_MOTOR_NODE_ID, can_cmd_request, owldrv::can_val_total_current, data);
}

void CanRobotDriver::requestPushboxState(){
  canDataType_t data;
  sendCanData(OWL_RECEIVER_MSG_ID, RECEIVER_PUSHBOX_NODE_ID, can_cmd_request, owlrecv::can_val_button_state, data);  
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
        int val = frame.data[3];            
        canDataType_t data;
        data.byteVal[0] = frame.data[4];
        data.byteVal[1] = frame.data[5];
        data.byteVal[2] = frame.data[6];
        data.byteVal[3] = frame.data[7];    

        switch (frame.can_id){
          case OWL_RECEIVER_MSG_ID:
            if (cmd == can_cmd_info){
              switch (val){
                case owlrecv::can_val_button_state:                  
                  if (data.intValue != 0){
                    CONSOLE.print("PUSHBOX: ");                   
                    CONSOLE.println(data.intValue, BIN);                                      
                    if (data.intValue == 1) setOperation(OP_MOW, false);
                    if (data.intValue == 2) Logger.event(EVT_AUDIO_SHEEP);                    
                    if (data.intValue == 4) setOperation(OP_DOCK, false);                                                          
                    if (data.intValue == 8) setOperation(OP_IDLE, false);
                    //if (data.intValue != 4){    // do not play any buzzer sound for pushbox STOP button
                      buzzer.sound(SND_READY, true);
                    //}
                  }  
                  //triggeredPushboxStopButton = (data.intValue == 4); // classic STOP button handling (pushbox STOP button simulates robot button)
                  break;
              }
            }              
            break;
          case OWL_DRIVE_MSG_ID:
            if (cmd == can_cmd_info){
                //CONSOLE.println("can_cmd_info");                
                // info value (volt, velocity, position, ...)
                switch (val){                            
                  case owldrv::can_val_error:
                    for (int i=0; i < MOW_MOTOR_COUNT; i++){
                      if (node.sourceAndDest.sourceNodeID == MOW_MOTOR_NODE_IDS[i]){
                        mowFault[i] = (data.byteVal[0] != err_ok);                        
                      }
                    }
                    switch(node.sourceAndDest.sourceNodeID){
                      case LEFT_MOTOR_NODE_ID:
                        leftMotorFault = (data.byteVal[0] != err_ok);
                        break;
                      case RIGHT_MOTOR_NODE_ID:
                        rightMotorFault = (data.byteVal[0] != err_ok);
                        break;
                    }                    
                    break;
                  case owldrv::can_val_total_current:
                    if (node.sourceAndDest.sourceNodeID == LEFT_MOTOR_NODE_ID) motorLeftCurr = data.floatVal;
                    if (node.sourceAndDest.sourceNodeID == RIGHT_MOTOR_NODE_ID) motorRightCurr = data.floatVal;
                    for (int i=0; i < MOW_MOTOR_COUNT; i++){
                      if (node.sourceAndDest.sourceNodeID == MOW_MOTOR_NODE_IDS[i]){
                        mowCurr[i] = data.floatVal;                        
                      }
                    }
                    break;
                  case owldrv::can_val_endswitch:
                    switch(node.sourceAndDest.sourceNodeID){
                      case MOW_HEIGHT_MOTOR_NODE_ID:  
                        if (data.byteVal[0] != 0){                          
                          if (!motorHeightFoundEndswitch){
                            motorHeightFoundEndswitch = true;
                            motorHeightFoundEndswitchTime = millis();
                            CONSOLE.println("******   found endswitch");                                                                              
                          }                                                
                        } 
                        break;
                    }
                    break;
                  case owldrv::can_val_angle:
                    switch(node.sourceAndDest.sourceNodeID){
                      case MOW_HEIGHT_MOTOR_NODE_ID:  
                        motorHeightAngleCurr = data.floatVal;
                        if ((motorHeightFoundEndswitch) && (!motorHeightAngleEndswitchSet) && (millis() > motorHeightFoundEndswitchTime + 4000)){
                          motorHeightAngleEndswitch= motorHeightAngleCurr; 
                          motorHeightAngleEndswitchSet = true;
                          CONSOLE.print("******   endswitch angle ");
                          CONSOLE.println(motorHeightAngleCurr);           
                        }                        
                        break;
                    }
                    break;
                  case owldrv::can_val_odo_ticks:
                    for (int i=0; i < MOW_MOTOR_COUNT; i++){
                      if (node.sourceAndDest.sourceNodeID == MOW_MOTOR_NODE_IDS[i]){
                        encoderTicksMow[i] = data.ofsAndByte.ofsVal;                        
                        //CONSOLE.print("mow encoder state ");
                        //CONSOLE.print(i);                                                
                        //CONSOLE.print(":");
                        //CONSOLE.println(encoderTicksMow[i]);
                        motorResponse();
                      }
                    }
                    switch(node.sourceAndDest.sourceNodeID){
                      case LEFT_MOTOR_NODE_ID:
                        //CONSOLE.println("encoderTicksLeft");
                        encoderTicksLeft = data.ofsAndByte.ofsVal;
                        //CONSOLE.print(encoderTicksLeft);
                        //CONSOLE.print(",");
                        //CONSOLE.println(data.ofsAndByte.ofsVal);                        
                        motorResponse();
                        break;
                      case RIGHT_MOTOR_NODE_ID:
                        encoderTicksRight = data.ofsAndByte.ofsVal;                        
                        motorResponse();
                        break;
                    }                
                    break;
                                
                }
            }
            break;
          case OWL_CONTROL_MSG_ID:
            if (cmd == can_cmd_info){
              switch (val){
                case owlctl::can_val_battery_voltage:
                  batteryVoltage = data.floatVal; 
                  /*
                  if (voltage > batteryVoltage + 0.5){
                    chargeVoltage = voltage;
                  } else if (voltage < batteryVoltage -0.5){                    
                    chargeVoltage = 0;                  
                  }
                  */
                  break;
                case owlctl::can_val_bumper_state:
                  triggeredLeftBumper = triggeredRightBumper = (data.byteVal[0] != 0);                  
                  break;
                case owlctl::can_val_stop_button_state:
                  triggeredStopButton = (data.byteVal[0] != 0);
                  //CONSOLE.println(triggeredStopButton);
                  break;
                case owlctl::can_val_rain_state:
                  triggeredRain = (data.byteVal[0] != 0);
                  break;
                case owlctl::can_val_slow_down_state:
                  triggeredSlowDown = (data.byteVal[0] != 0);
                  break;
                case owlctl::can_val_lift_state:
                  triggeredLift = (data.byteVal[0] != 0);
                  break;
                case owlctl::can_val_charger_voltage:
                  float volt = data.floatVal;                  
                  //CONSOLE.print("charger: ");
                  //CONSOLE.println(volt);                                    
                  if (volt > 1000) volt = volt / 1000.0;
                  if (volt > 20) chargeVoltage = volt;
                    else chargeVoltage = 0;            
                  break;
              }
            }
            break;

        } 
    }
  }
}

void CanRobotDriver::run(){  
  processResponse();
  if (millis() > nextMotorTime){
    nextMotorTime = millis() + 20; // 50 hz
    /*while (can.available()){
      can_frame_t frame;
      can.read(frame);
    }*/
    //CONSOLE.println(requestLeftPwm);
    requestMotorDrivePwm(requestLeftPwm, requestRightPwm, requestReleaseBrakesWhenZero);        
  }
  if (millis() > nextSummaryTime){
    nextSummaryTime = millis() + 100; // 10 hz
    requestSummary();
  }
  if (millis() > nextCheckErrorTime){
    nextCheckErrorTime = millis() + 2000; // 0.5 hz
    requestMotorErrorStatus();
  }  
  if (millis() > nextMowTime){
    nextMowTime = millis() + 300;  // 3 hz      
    requestMotorMowPwm(requestMowPwm);
    requestPushboxState();
  }
  if (millis() > nextConsoleTime){
    nextConsoleTime = millis() + 1000;  // 1 hz    
    requestMotorMowCurrent();
    if (MOW_ADJUST_HEIGHT){   // can the mowing height be adjusted by an additional motor?
      requestMowHeight(requestMowHeightMillimeter);
    }    
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
  for (int i=0; i < MOW_MOTOR_COUNT; i++) lastEncoderTicksMow[i] = 0;
}

void CanMotorDriver::run(){
}

void CanMotorDriver::setMowHeight(int mowHeightMillimeter){
  canRobot.requestMowHeightMillimeter = mowHeightMillimeter;
}

void CanMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm, bool releaseBrakesWhenZero){  
  //CONSOLE.print("CanMotorDriver::setMotorPwm ");  
  //CONSOLE.print(leftPwm);
  //CONSOLE.print(",");  
  //CONSOLE.print(rightPwm);
  //CONSOLE.print(",");  
  //CONSOLE.println(mowPwm);
  //canRobot.requestMotorPwm(leftPwm, rightPwm, mowPwm);
  canRobot.requestLeftPwm = leftPwm;
  canRobot.requestRightPwm = rightPwm;
  canRobot.requestReleaseBrakesWhenZero = releaseBrakesWhenZero;
  // Alfred mowing motor driver seem to start start mowing motor more successfully with full PWM (100%) values...  
  //if (mowPwm > 0) mowPwm = 255;
  //  else if (mowPwm < 0) mowPwm = -255;
  canRobot.requestMowPwm = mowPwm;
}

void CanMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){
  leftFault = canRobot.leftMotorFault;
  rightFault = canRobot.rightMotorFault;
  mowFault = false;
  if (MOW_MOTOR_COUNT > 0) mowFault = canRobot.mowFault[0];
  for (int i=0; i < MOW_MOTOR_COUNT; i++) mowFault = (mowFault && canRobot.mowFault[i]);
  if ( (mowFault) || (leftFault) || (rightFault) ){
    CONSOLE.print("canRobot: motorFault (lefErr=");
    CONSOLE.print(leftFault);
    CONSOLE.print(" rightErr=");
    CONSOLE.print(rightFault);
    CONSOLE.print(" mowErr=");
    CONSOLE.print(mowFault);
    CONSOLE.println(")");
  }
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
  mowCurrent = 0;
  for (int i=0; i < MOW_MOTOR_COUNT; i++){
    mowCurrent = max(mowCurrent, canRobot.mowCurr[i]);
  }
}

void CanMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  //CONSOLE.println("getMotorEncoderTicks");    
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
    for (int i=0; i < MOW_MOTOR_COUNT; i++) lastEncoderTicksMow[i] = canRobot.encoderTicksMow[i];
  }
  leftTicks = (unsigned short)(canRobot.encoderTicksLeft - lastEncoderTicksLeft);
  rightTicks = (unsigned short)(canRobot.encoderTicksRight - lastEncoderTicksRight);
  
  int allMowTicks[MOW_MOTOR_COUNT];
  mowTicks = 0; // 99999
  //for (int i=0; i < 1; i++){  
  for (int i=0; i < MOW_MOTOR_COUNT; i++){
    allMowTicks[i] = (unsigned short)(canRobot.encoderTicksMow[i] - lastEncoderTicksMow[i]);
    //CONSOLE.print("allMowTicks ");
    //CONSOLE.print(i);
    //CONSOLE.print(":");
    //CONSOLE.println(allMowTicks[i]);
    if (allMowTicks[i] > 5000) {
      CONSOLE.println("CanMotorDriver::getMotorEncoderTicks resetting mowTicks");      
      allMowTicks[i] = 0;
    }
    lastEncoderTicksMow[i] = canRobot.encoderTicksMow[i];
    mowTicks = max(mowTicks, allMowTicks[i]);  // just consider one motor (with overall minimum ticks)  
  }
  //CONSOLE.print("mowTicks ");
  //CONSOLE.println(mowTicks);  

  if (leftTicks > 5000){
    CONSOLE.println("CanMotorDriver::getMotorEncoderTicks resetting leftTicks");
    leftTicks = 0;
  }
  if (rightTicks > 5000){
    CONSOLE.println("CanMotorDriver::getMotorEncoderTicks resetting rightTicks");
    rightTicks = 0;
  } 
  lastEncoderTicksLeft = canRobot.encoderTicksLeft;
  lastEncoderTicksRight = canRobot.encoderTicksRight;  
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

bool CanBumperDriver::nearObstacle(){
  return canRobot.triggeredSlowDown;
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
  return (canRobot.triggeredStopButton) || (canRobot.triggeredPushboxStopButton); 
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
  canDataType_t data;
  data.byteVal[0] = 0;  
  canRobot.sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_set, owlctl::can_val_buzzer_state, data );
}

void CanBuzzerDriver::tone(int freq){
  canDataType_t data;
  data.byteVal[0] = 1;  
  canRobot.sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_set, owlctl::can_val_buzzer_state, data );
}



