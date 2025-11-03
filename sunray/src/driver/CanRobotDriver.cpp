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
#ifdef __linux__
  #include <fcntl.h>
  #include <sys/mman.h>
  #include <signal.h>
#endif

//#define COMM  ROBOT

//#define DEBUG_CAN_ROBOT 1

#ifndef POWER_OFF_COMMAND_DELAY_SEC
#define POWER_OFF_COMMAND_DELAY_SEC 30
#endif

#ifndef POWER_OFF_LOG_INTERVAL_MS
#define POWER_OFF_LOG_INTERVAL_MS 2000
#endif

#ifndef POWER_OFF_COMMAND_RETRY_MS
#define POWER_OFF_COMMAND_RETRY_MS 1000
#endif

#ifndef POWER_OFF_DECISION_TIMEOUT_MS
#define POWER_OFF_DECISION_TIMEOUT_MS 30000UL
#endif

#ifndef POWER_OFF_GPIO_CONFIRMATION_SECONDS
#define POWER_OFF_GPIO_CONFIRMATION_SECONDS 3
#endif

#ifndef SIMULATE_SUNRAY_POWER_OFF_HANG
#define SIMULATE_SUNRAY_POWER_OFF_HANG false
#endif

#ifndef SIMULATE_SUNRAY_POWER_OFF_HANG_DURATION_SEC
#define SIMULATE_SUNRAY_POWER_OFF_HANG_DURATION_SEC 20  // simulated update duration in seconds
#endif

#ifndef SIMULATE_SUNRAY_POWER_OFF_COMMAND_DELAY_SEC
#define SIMULATE_SUNRAY_POWER_OFF_COMMAND_DELAY_SEC 3     // delay before sending power-off command to owlController
#endif

static const char* powerOffStateToStr(owlctl::powerOffState_t state){
  switch (state){
    case owlctl::power_off_inactive: return "inactive";
    case owlctl::power_off_active: return "active";
    case owlctl::power_off_shutdown_pending: return "shutdown_pending";
    default: return "unknown";
  }
}

int MOW_MOTOR_NODE_IDS[] = { MOW1_MOTOR_NODE_ID, MOW2_MOTOR_NODE_ID, MOW3_MOTOR_NODE_ID, MOW4_MOTOR_NODE_ID, MOW5_MOTOR_NODE_ID  };
int OWL_MSG_IDS[] = { OWL_RECEIVER_MSG_ID, OWL_CONTROL_MSG_ID, OWL_DRIVE_MSG_ID, OWL_RELAIS_MSG_ID };


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
  nextDisplayStateTime = 0;
  lastDisplayOpSent = stateEstimator.stateOp;
  lastIpSent = "";
  lastIpSentValid = false;
  for (uint8_t &b : lastIpSentBytes) {
    b = 0;
  }
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
  nextDisplayTelemetryTime = 0;
  nextIpTime = 0;
  lastWifiSignalDbm = -127;
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
  powerOffLogTime = 0;
  powerOffCommandSendTime = 0;
  powerOffCommandSent = false;
  powerOffCommandAccepted = false;
  linuxShutdownIssued = false;
  powerOffState = owlctl::power_off_inactive;
  powerOffDelaySeconds = POWER_OFF_COMMAND_DELAY_SEC;
  powerOffDecisionPending = false;
  powerOffDecisionStartTime = 0;
  powerOffDecisionDeadline = 0;
  powerOffDecisionDelaySeconds = POWER_OFF_COMMAND_DELAY_SEC;
  powerOffDecisionTrigger = PowerOffDecisionTrigger::None;
  simulatePowerOffHang = false;
  simulatePowerOffHangNotified = false;
  simulatePowerOffHangUntil = 0;
  simulatePowerOffHangConfigured = SIMULATE_SUNRAY_POWER_OFF_HANG;
  simulatePowerOffHangConfiguredDuration = SIMULATE_SUNRAY_POWER_OFF_HANG_DURATION_SEC;
  simulatePowerOffHangCommandPending = false;
  simulatePowerOffHangCommandTime = 0;
  simulatePiSelfShutdownPending = false;
  simulatePiSelfShutdownTime = 0;
  satSummarySent = false;
  lastSatStatus = 0xFF;
  lastSatUsed = 0xFF;
  lastSatTotal = 0xFF;
  rtkAgeSent = false;
  lastRtkAgeTenths = 0xFFFF;
  mapProgressSent = false;
  lastMapCount = 0xFFFF;
  lastMapPercent = 0xFF;

  #ifdef __linux__
    Process p;
    p.runShellCommand("pwd");
	  String workingDir = p.readString();    
    CONSOLE.print("linux working dir (pwd): ");
    CONSOLE.println(workingDir);

    CONSOLE.println("reading robot ID...");
    Process p2;    
    p2.runShellCommand("ip link show wlan0 | grep link/ether | awk '{print $2}'");
	  robotID = p2.readString();
    robotID.trim();
    if (robotID == ""){
      p2.runShellCommand("ip link show eth0 | grep link/ether | awk '{print $2}'");
      robotID = p2.readString();
      robotID.trim();  
    }    
    
    if (false){
      // trigger linux bus error - steps to examine:
      // 1. (optional) set core pattern:  sudo sysctl -w 'kernel.core_pattern=|/usr/lib/systemd/systemd-coredump %P %u %g %s %t 9223372036854775808 %h'      
      // 2. activate coredumps:     ulimit -c unlimited
      // 3. start sunray until crash
      // 4. list coredumps:         coredumpctl list   (and look for PID, e.g 144840) 
      // 5. extract coredump:       sudo coredumpctl dump 144840 --output 144840.core
      // 6. find reason for crash:  gdb build/sunray -c 144840.core   (and press ENTER, ENTER)
      CONSOLE.println("WARN: simulating a bus error!");
      const char* filename = "testfile.bin";
      int fd = open(filename, O_RDWR | O_CREAT | O_TRUNC, 0666);
      ftruncate(fd, 4096);  
      char* data = (char*)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      close(fd);  
      truncate(filename, 0);
      data[0] = 'X';
      munmap(data, 4096);
    }  
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

void CanRobotDriver::updateWifiSignalStrength(){
#if ENABLE_CAN_DISPLAY
  #ifdef __linux__
    String result;
    while (wifiSignalProcess.available()) result += (char)wifiSignalProcess.read();
    result.trim();

    int16_t newDbm = lastWifiSignalDbm;
    bool haveSample = false;

    if (result.length() > 0) {
      if (result.startsWith("-") || isDigit(result.charAt(0))) {
        int parsed = result.toInt();
        if (!(parsed == 0 && result.indexOf('-') == -1)) {
          newDbm = static_cast<int16_t>(parsed);
          haveSample = true;
        }
      }
    } else {
      newDbm = -127;
      haveSample = true;
    }

    if (haveSample) {
      canDataType_t data;
      data.floatVal = static_cast<float>(newDbm);
      sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_wifi_signal, data);
      lastWifiSignalDbm = newDbm;
    }

    wifiSignalProcess.runShellCommand("iw dev wlan0 link | awk '/signal/ {print $2}'");
  #endif
#endif
}

void CanRobotDriver::updateDisplayTelemetry(){
#if !ENABLE_CAN_DISPLAY
  return;
#else
  const unsigned long now = millis();

  uint8_t satStatus = 2;
  switch (gps.solution) {
    case SOL_FIXED: satStatus = 0; break;
    case SOL_FLOAT: satStatus = 1; break;
    default: satStatus = 2; break;
  }

  int totalRaw = gps.numSV;
  if (totalRaw < 0) totalRaw = 0;
  uint8_t satTotal = static_cast<uint8_t>(min(totalRaw, 255));

  int usedRaw = gps.numSVdgps >= 0 ? gps.numSVdgps : gps.numSV;
  if (gps.solution == SOL_INVALID) {
    usedRaw = 0;
  }
  if (usedRaw < 0) usedRaw = 0;
  if (usedRaw > totalRaw) usedRaw = totalRaw;
  uint8_t satUsed = static_cast<uint8_t>(min(usedRaw, 255));

  if (!satSummarySent ||
      satStatus != lastSatStatus ||
      satUsed != lastSatUsed ||
      satTotal != lastSatTotal) {
    canDataType_t data;
    data.intValue = 0;
    data.byteVal[0] = satStatus;
    data.byteVal[1] = satUsed;
    data.byteVal[2] = satTotal;
    data.byteVal[3] = 0;
    sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_sat_summary, data);
    satSummarySent = true;
    lastSatStatus = satStatus;
    lastSatUsed = satUsed;
    lastSatTotal = satTotal;
  }

  float ageSeconds;
  if (gps.dgpsAge == 0) {
    ageSeconds = 9999.0f;
  } else {
    unsigned long ageMs = now - gps.dgpsAge;
    ageSeconds = ageMs / 1000.0f;
  }
  if (ageSeconds < 0.0f) ageSeconds = 0.0f;
  if (ageSeconds > 9999.0f) ageSeconds = 9999.0f;
  uint16_t ageTenths = static_cast<uint16_t>(min(65535UL, (unsigned long)roundf(ageSeconds * 10.0f)));

  if (!rtkAgeSent || ageTenths != lastRtkAgeTenths) {
    canDataType_t data;
    data.floatVal = ageSeconds;
    sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_rtk_age, data);
    rtkAgeSent = true;
    lastRtkAgeTenths = ageTenths;
  }

  bool mapAvailable = (maps.mowPoints.numPoints > 0);
  if (mapAvailable) {
    int mapCountRaw = maps.mowPointsIdx;
    if (mapCountRaw < 0) mapCountRaw = 0;
    uint16_t mapCount = static_cast<uint16_t>(min(mapCountRaw, 0xFFFF));
    int percentRaw = maps.percentCompleted;
    if (percentRaw < 0) percentRaw = 0;
    if (percentRaw > 100) percentRaw = 100;
    uint8_t mapPercent = static_cast<uint8_t>(percentRaw);

    if (!mapProgressSent || mapCount != lastMapCount || mapPercent != lastMapPercent) {
      canDataType_t data;
      data.byteVal[0] = mapCount & 0xFF;
      data.byteVal[1] = (mapCount >> 8) & 0xFF;
      data.byteVal[2] = mapPercent;
      data.byteVal[3] = 0;
      sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_map_progress, data);
      mapProgressSent = true;
      lastMapCount = mapCount;
      lastMapPercent = mapPercent;
    }
  } else {
    mapProgressSent = false;
    lastMapCount = 0xFFFF;
    lastMapPercent = 0xFF;
  }
#endif
}


void CanRobotDriver::sendIpAddress(){
  #ifdef __linux__
    while (ipAddressToStringProcess.available()) ipAddressToStringProcess.read();

    // Use hostname -I to avoid netlink permission issues; first token is primary IPv4
    ipAddressToStringProcess.runShellCommand("hostname -I");
    String ipStr = ipAddressToStringProcess.readString();
    ipStr.trim();

    if (ipStr.length() == 0) {
        return;
    }

    int spacePos = ipStr.indexOf(' ');
    if (spacePos > 0) {
        ipStr = ipStr.substring(0, spacePos);
    }

    IPAddress ip;
    if (!ip.fromString(ipStr)) {
        return;
    }

    if (lastIpSentValid) {
        bool unchanged = true;
        for (uint8_t i = 0; i < 4; i++) {
            if (ip[i] != lastIpSentBytes[i]) {
                unchanged = false;
                break;
            }
        }
        if (unchanged) {
            return; // already displayed
        }
    }

    canDataType_t data;
    for (uint8_t i = 0; i < 4; i++) {
        data.byteVal[i] = ip[i];
        lastIpSentBytes[i] = ip[i];
    }

    sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_set, owlctl::can_val_ip_address, data);
#if ENABLE_CAN_DISPLAY
    sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_ip_address, data);
#endif
    lastIpSent = ipStr;
    lastIpSentValid = true;
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

void CanRobotDriver::sendDisplayOperation(OperationType op){
    owldisplay::stateCode_t code = owldisplay::state_unknown;
    switch (op){
        case OP_MOW:    code = owldisplay::state_mow;    break;
        case OP_DOCK:   code = owldisplay::state_dock;   break;
        case OP_IDLE:   code = owldisplay::state_idle;   break;
        case OP_CHARGE: code = owldisplay::state_charge; break;
        case OP_ERROR:  code = owldisplay::state_error;  break;
        default:        code = owldisplay::state_unknown; break;
    }
  canDataType_t data;
  data.intValue = 0;
  data.byteVal[0] = static_cast<uint8_t>(code);
  lastDisplayOpSent = op;
#if ENABLE_CAN_DISPLAY
  sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_state_code, data);
#endif
}



// request MCU SW version
void CanRobotDriver::requestVersion(){
}


// request MCU summary
void CanRobotDriver::requestSummary(){
  canDataType_t data;
  data.floatVal = 0;
  
  switch (cmdSummaryCounter % 8){
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
    case 7:
      requestPowerOffState();
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

void CanRobotDriver::requestPowerOffState(){
  canDataType_t data;
  sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_request, owlctl::can_val_power_off_state, data);
}

void CanRobotDriver::requestManagedShutdown(uint8_t delaySeconds){
  powerOffDelaySeconds = delaySeconds;
  startPowerOffDecision(delaySeconds, PowerOffDecisionTrigger::InternalRequest);
}

void CanRobotDriver::sendPowerOffCommand(uint8_t delaySeconds){
  canDataType_t data;
  data.byteVal[0] = delaySeconds;
  data.byteVal[1] = 0;
  data.byteVal[2] = 0;
  data.byteVal[3] = 0;
  sendCanData(OWL_CONTROL_MSG_ID, CONTROL_NODE_ID, can_cmd_set, owlctl::can_val_power_off_command, data);
  powerOffCommandSent = true;
  powerOffCommandAccepted = false;
  powerOffCommandSendTime = millis();
  unsigned long now = millis();
  if (now > powerOffLogTime){
    CONSOLE.print("CAN: requested power-off in ");
    CONSOLE.print(delaySeconds);
    CONSOLE.println("s");
    powerOffLogTime = now + POWER_OFF_LOG_INTERVAL_MS;
  }
  if (simulatePowerOffHang){
    simulatePowerOffHangCommandPending = false;
    simulatePowerOffHangCommandTime = 0;
    if ((simulatePowerOffHangConfiguredDuration > 0) && (simulatePowerOffHangUntil == 0)){
      simulatePowerOffHangUntil = powerOffCommandSendTime + simulatePowerOffHangConfiguredDuration * 1000UL;
      simulatePiSelfShutdownPending = true;
      simulatePiSelfShutdownTime = simulatePowerOffHangUntil;
    }
  }
}

uint8_t CanRobotDriver::getPowerOffDelaySeconds() const{
  return powerOffDelaySeconds;
}

void CanRobotDriver::setSimulatePowerOffHang(bool flag, unsigned long durationSeconds){
  unsigned long now = millis();
  bool previous = simulatePowerOffHang;
  if (flag){
    simulatePowerOffHang = true;
    simulatePowerOffHangNotified = false;
    if (durationSeconds > 0){
      simulatePowerOffHangConfiguredDuration = durationSeconds;
    }
    simulatePowerOffHangUntil = 0;
    simulatePiSelfShutdownPending = false;
    simulatePiSelfShutdownTime = 0;
    simulatePowerOffHangCommandPending = true;
    simulatePowerOffHangCommandTime = now + (unsigned long)SIMULATE_SUNRAY_POWER_OFF_COMMAND_DELAY_SEC * 1000UL;
    if (!previous){
      if (simulatePowerOffHangConfiguredDuration > 0){
        CONSOLE.print("CAN: simulate power-off hang enabled for ");
        CONSOLE.print(simulatePowerOffHangConfiguredDuration);
        CONSOLE.println("s");
      } else {
        CONSOLE.println("CAN: simulate power-off hang enabled (indefinite)");
      }
    } else if (simulatePowerOffHangConfiguredDuration > 0){
      CONSOLE.print("CAN: simulate power-off hang extended for ");
      CONSOLE.print(simulatePowerOffHangConfiguredDuration);
      CONSOLE.println("s");
    }
  } else {
    simulatePowerOffHang = false;
    simulatePowerOffHangNotified = false;
    simulatePowerOffHangUntil = 0;
    simulatePowerOffHangCommandPending = false;
    simulatePowerOffHangCommandTime = 0;
    simulatePiSelfShutdownPending = false;
    simulatePiSelfShutdownTime = 0;
    if (previous){
      CONSOLE.println("CAN: simulate power-off hang disabled");
    }
  }
}

bool CanRobotDriver::getSimulatePowerOffHang() const{
  return simulatePowerOffHang;
}

void CanRobotDriver::versionResponse(){
}


void CanRobotDriver::summaryResponse(){
}

void CanRobotDriver::handlePowerOffState(owlctl::powerOffState_t remoteState, uint8_t activeSeconds, uint8_t configuredDelay){
  if (configuredDelay != 0){
    powerOffDelaySeconds = configuredDelay;
  } else {
    powerOffDelaySeconds = POWER_OFF_COMMAND_DELAY_SEC;
  }
  bool stateChanged = (powerOffState != remoteState);
  powerOffState = remoteState;
  unsigned long now = millis();
  if (stateChanged && now > powerOffLogTime){
    CONSOLE.print("CAN: power-off state -> ");
    CONSOLE.print(powerOffStateToStr(remoteState));
    CONSOLE.print(" (held ");
    CONSOLE.print(activeSeconds);
    CONSOLE.println("s)");
    powerOffLogTime = now + POWER_OFF_LOG_INTERVAL_MS;
  }
  switch (remoteState){
    case owlctl::power_off_active:
      if (activeSeconds >= POWER_OFF_GPIO_CONFIRMATION_SECONDS){
        if (!powerOffDecisionPending || powerOffDecisionTrigger != PowerOffDecisionTrigger::ExternalPin){
          startPowerOffDecision(powerOffDelaySeconds, PowerOffDecisionTrigger::ExternalPin);
        }
      } else {
        cancelPowerOffDecision(PowerOffDecisionTrigger::ExternalPin);
      }
      break;
    case owlctl::power_off_inactive:
      cancelPowerOffDecision(PowerOffDecisionTrigger::ExternalPin);
      if (!powerOffCommandAccepted && (powerOffCommandSendTime != 0)){
        CONSOLE.println("CAN: power-off pin released before shutdown ack");
      }
      powerOffCommandSent = false;
      powerOffCommandAccepted = false;
      linuxShutdownIssued = false;
      powerOffCommandSendTime = 0;
      break;
    case owlctl::power_off_shutdown_pending:
      break;
  }
}

void CanRobotDriver::handlePowerOffCommandAck(uint8_t acceptedFlag, uint8_t delaySeconds){
  bool accepted = (acceptedFlag != 0);
  if (accepted){
    if (delaySeconds != 0){
      powerOffDelaySeconds = delaySeconds;
    }
    powerOffCommandSent = true;
    if (!powerOffCommandAccepted){
      powerOffCommandAccepted = true;
      powerOffState = owlctl::power_off_shutdown_pending;
      unsigned long now = millis();
      if (now > powerOffLogTime){
        CONSOLE.print("CAN: power-off accepted, shutdown in ");
        CONSOLE.print(powerOffDelaySeconds);
        CONSOLE.println("s");
        powerOffLogTime = now + POWER_OFF_LOG_INTERVAL_MS;
      }
      ledStateShutdown = true;
      #ifdef __linux__
        if (simulatePowerOffHang && (simulatePowerOffHangConfiguredDuration > 0)){
          if (!simulatePiSelfShutdownPending){
            unsigned long target = powerOffCommandSendTime + simulatePowerOffHangConfiguredDuration * 1000UL;
            if ((simulatePowerOffHangUntil != 0) && (simulatePowerOffHangUntil > target)){
              target = simulatePowerOffHangUntil;
            }
            simulatePiSelfShutdownPending = true;
            simulatePiSelfShutdownTime = target;
          }
        } else if (!linuxShutdownIssued){
          linuxShutdownIssued = true;
          Process p;
          p.runShellCommand("shutdown now");
          CONSOLE.println("CAN: linux shutdown command issued");
        }
      #endif
    }
  } else {
    if (powerOffCommandSent){
      unsigned long now = millis();
      if (now > powerOffLogTime){
        CONSOLE.println("CAN: power-off command not accepted");
        powerOffLogTime = now + POWER_OFF_LOG_INTERVAL_MS;
      }
    }
    powerOffCommandSent = false;
    powerOffCommandAccepted = false;
    linuxShutdownIssued = false;
    powerOffCommandSendTime = 0;
  }
}

void CanRobotDriver::startPowerOffDecision(uint8_t delaySeconds, PowerOffDecisionTrigger trigger){
  unsigned long now = millis();
  bool freshTrigger = (!powerOffDecisionPending) || (powerOffDecisionTrigger != trigger);
  if (freshTrigger){
    powerOffDecisionStartTime = now;
    powerOffDecisionDeadline = now + POWER_OFF_DECISION_TIMEOUT_MS;
  }
  powerOffDecisionPending = true;
  powerOffDecisionTrigger = trigger;
  powerOffDecisionDelaySeconds = delaySeconds;
  if (freshTrigger && simulatePowerOffHangConfigured){
    setSimulatePowerOffHang(true, simulatePowerOffHangConfiguredDuration);
  }
  if (freshTrigger && now > powerOffLogTime){
    const char* triggerStr = "none";
    switch (trigger){
      case PowerOffDecisionTrigger::ExternalPin:
        triggerStr = "pin";
        break;
      case PowerOffDecisionTrigger::InternalRequest:
        triggerStr = "sunray";
        break;
      default:
        break;
    }
    CONSOLE.print("CAN: preparing shutdown (trigger=");
    CONSOLE.print(triggerStr);
    CONSOLE.print(") with delay ");
    CONSOLE.print(delaySeconds);
    CONSOLE.println("s");
    powerOffLogTime = now + POWER_OFF_LOG_INTERVAL_MS;
  }
}

void CanRobotDriver::cancelPowerOffDecision(PowerOffDecisionTrigger trigger){
  if (!powerOffDecisionPending){
    return;
  }
  if ((trigger != PowerOffDecisionTrigger::None) && (powerOffDecisionTrigger != trigger)){
    return;
  }
  powerOffDecisionPending = false;
  powerOffDecisionTrigger = PowerOffDecisionTrigger::None;
  powerOffDecisionStartTime = 0;
  powerOffDecisionDeadline = 0;
  simulatePowerOffHangNotified = false;
  if (simulatePowerOffHang){
    setSimulatePowerOffHang(false);
  }
}

bool CanRobotDriver::readyForManagedShutdown(PowerOffDecisionTrigger trigger){
  (void)trigger;
  return true;
}

void CanRobotDriver::processPowerOffDecision(){
  if (!powerOffDecisionPending){
    simulatePowerOffHangNotified = false;
    return;
  }
  unsigned long now = millis();
  if (simulatePowerOffHang){
    if (!simulatePowerOffHangNotified){
      CONSOLE.println("CAN: simulation active - delaying power-off command");
      simulatePowerOffHangNotified = true;
    }
    if (simulatePowerOffHangCommandPending){
      return;
    }
  }
  simulatePowerOffHangNotified = false;
  bool timeoutReached = (powerOffDecisionDeadline != 0) && (now >= powerOffDecisionDeadline);
  if (readyForManagedShutdown(powerOffDecisionTrigger) || timeoutReached){
    sendPowerOffCommand(powerOffDecisionDelaySeconds);
    powerOffDecisionPending = false;
    powerOffDecisionTrigger = PowerOffDecisionTrigger::None;
    powerOffDecisionStartTime = 0;
    powerOffDecisionDeadline = 0;
  }
}

void CanRobotDriver::processPendingPowerOffCommand(){
  if (simulatePowerOffHang){
    return;
  }
  if (!powerOffCommandSent || powerOffCommandAccepted){
    return;
  }
  unsigned long now = millis();
  if ((powerOffCommandSendTime == 0) || (now - powerOffCommandSendTime > POWER_OFF_COMMAND_RETRY_MS)){
    sendPowerOffCommand(powerOffDelaySeconds);
  }
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
                    {
#if ENABLE_CAN_DISPLAY
                      canDataType_t displayData;
                      float totalCurrent = motorLeftCurr + motorRightCurr;
                      for (int i = 0; i < MOW_MOTOR_COUNT; i++) totalCurrent += mowCurr[i];
                      displayData.floatVal = totalCurrent;
                      sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_battery_current, displayData);
#endif
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
                case owlctl::can_val_battery_voltage: {
                  batteryVoltage = data.floatVal; 

#if ENABLE_CAN_DISPLAY
                  canDataType_t displayData;
                  displayData.floatVal = batteryVoltage;
                  sendCanData(OWL_DISPLAY_MSG_ID, DISPLAY_NODE_ID, can_cmd_info, owldisplay::can_val_battery_voltage, displayData);
#endif
                  /*
                  if (voltage > batteryVoltage + 0.5){
                    chargeVoltage = voltage;
                  } else if (voltage < batteryVoltage -0.5){                    
                    chargeVoltage = 0;                  
                  }
                  */
                  break;
                }
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
                case owlctl::can_val_charger_voltage: {
                  float volt = data.floatVal;                  
                  //CONSOLE.print("charger: ");
                  //CONSOLE.println(volt);                                    
                  if (volt > 1000) volt = volt / 1000.0;
                  if (volt > 20) chargeVoltage = volt;
                    else chargeVoltage = 0;            
                  break;
                }
                case owlctl::can_val_power_off_state:
                  handlePowerOffState((owlctl::powerOffState_t)data.byteVal[0], data.byteVal[1], data.byteVal[2]);
                  break;
                case owlctl::can_val_power_off_command:
                  handlePowerOffCommandAck(data.byteVal[0], data.byteVal[1]);
                  break;
              }
            }
            break;

              }
            }
  }
}

void CanRobotDriver::run(){  
  unsigned long now = millis();
  if (simulatePowerOffHangCommandPending && now >= simulatePowerOffHangCommandTime){
    simulatePowerOffHangCommandPending = false;
    simulatePowerOffHangCommandTime = 0;
    sendPowerOffCommand(powerOffDecisionDelaySeconds);
    powerOffDecisionPending = false;
    powerOffDecisionTrigger = PowerOffDecisionTrigger::None;
    powerOffDecisionStartTime = 0;
    powerOffDecisionDeadline = 0;
    simulatePowerOffHangNotified = false;
  }
  if (simulatePiSelfShutdownPending && now >= simulatePiSelfShutdownTime){
    simulatePiSelfShutdownPending = false;
    simulatePiSelfShutdownTime = 0;
    if (!linuxShutdownIssued){
      linuxShutdownIssued = true;
      #ifdef __linux__
        Process p;
        p.runShellCommand("shutdown now");
        CONSOLE.println("CAN: simulated update complete - issuing linux shutdown");
      #else
        CONSOLE.println("CAN: simulated update complete - linux shutdown requested (mock)");
      #endif
    }
  }
  if (simulatePowerOffHang && (simulatePowerOffHangUntil != 0) && now >= simulatePowerOffHangUntil){
    setSimulatePowerOffHang(false);
    simulatePowerOffHangNotified = false;
    CONSOLE.println("CAN: simulated update finished - power-off responses re-enabled");
  }
  processResponse();
  processPowerOffDecision();
  processPendingPowerOffCommand();
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
  if (millis() > nextDisplayStateTime){
    nextDisplayStateTime = millis() + 1000;  // refresh every second
    if (stateEstimator.stateOp != lastDisplayOpSent){
      lastDisplayOpSent = stateEstimator.stateOp;
      sendDisplayOperation(lastDisplayOpSent);
    }
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
  if (millis() > nextDisplayTelemetryTime){
    nextDisplayTelemetryTime = millis() + 2000;
    updateDisplayTelemetry();
  }
  if (millis() > nextWifiTime){
    nextWifiTime = millis() + 7000; // 7 sec
    updateWifiConnectionState();
    updateWifiSignalStrength();
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
  owlPowerOffNotified = false;
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
  if (flag){
    owlPowerOffNotified = false;
  } else if (!owlPowerOffNotified){
    canRobot.requestManagedShutdown(canRobot.getPowerOffDelaySeconds());
    owlPowerOffNotified = true;
  }

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

// ------------------------------------------------------------------------------------

CanRelaisDriver::CanRelaisDriver(CanRobotDriver &sr): canRobot(sr){
}

void CanRelaisDriver::begin(){
}

void CanRelaisDriver::run(){
}

void CanRelaisDriver::setRelaisState(int relais_node_id, bool state){
  canDataType_t data;
  if (state) data.byteVal[0] = 1;
  else data.byteVal[0] = 0;

  canRobot.sendCanData(OWL_RELAIS_MSG_ID, relais_node_id, can_cmd_set, owlrls::can_val_relais_state, data);
  CONSOLE.print(" Relais is now set to");
  CONSOLE.println(state);
}


bool CanRelaisDriver::getRelaisState(int relais_node_id){
  canDataType_t data;
  bool state = false;

  canRobot.sendCanData(OWL_RELAIS_MSG_ID, relais_node_id, can_cmd_request, owlrls::can_val_relais_state, data);

  if (data.byteVal[0] == 1) state = true;
  else state = false;

  return state;
}

void CanRelaisDriver::setRelaisStateCountdown(int relais_node_id, bool state, unsigned long countdown){
  canDataType_t data;
  data.intValue = countdown;

  canRobot.sendCanData(OWL_RELAIS_MSG_ID, relais_node_id, can_cmd_set, owlrls::can_val_relais_countdown, data);
}
