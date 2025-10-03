// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// robot communication interface (Console/Bluetooth/WIFI)

#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
#include "config.h"

class Comm {
public:
  // main entrypoints
  void processComm();
  void outputConsole();
  void processCmd(String channel, bool checkCrc, bool decrypt, bool verbose);
  void processConsole();
  void cmdSwitchOffRobot();

  // helpers for external producers/consumers
  void setCmd(const String &s) { cmd = s; }
  String getCmdResponse() const { return cmdResponse; }

private:
  // state previously global
  unsigned long nextInfoTime = 0;
  bool triggerWatchdog = false;

  int encryptMode = 0; // 0=off, 1=encrypt
  int encryptPass = PASS; 
  int encryptChallenge = 0;
  int encryptKey = 0;

  bool simFaultyConn = false; // simulate a faulty connection?
  int simFaultConnCounter = 0;

  String cmd;
  String cmdResponse;

  float statControlCycleTime = 0; 
  float statMaxControlCycleTime = 0; 

  // helpers
  void cmdAnswer(String s);
  void cmdTuneParam();
  void cmdControl();
  void cmdMotor();
  void cmdMotorTest();
  void cmdMotorPlot();
  void cmdSensorTest();
  void cmdTimetable();
  void cmdWaypoint();
  void cmdWayCount();
  void cmdExclusionCount();
  void cmdPosMode();
  void cmdVersion();
  void cmdObstacle();
  void cmdRain();
  void cmdBatteryLow();
  void cmdStressTest();
  void cmdTriggerWatchdog();
  void cmdGNSSReboot();
  void cmdKidnap();
  void cmdToggleGPSSolution();
  void cmdObstacles();
  void cmdSummary();
  void cmdStats();
  void cmdClearStats();
  void cmdWiFiScan();
  void cmdWiFiSetup();
  void cmdWiFiStatus();
  void cmdFirmwareUpdate();
};

extern Comm comm;



#endif
