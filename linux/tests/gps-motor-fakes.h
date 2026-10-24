// Hardware boundary for the production Motor implementation in startup tests.
#pragma once
#include "src/ublox/ublox.h"
extern UBLOX gps;
struct TestMotorDriver {
  int left=0,right=0,mow=0;
  bool release=false;
  void setMotorPwm(int l,int r,int m,bool b){left=l;right=r;mow=m;release=b;}
  void setMowHeight(int){}
  void getMotorEncoderTicks(int& l,int& r,int& m){l=r=m=0;}
  void resetMotorFaults(){}
  void getMotorFaults(bool& l,bool& r,bool& m){l=r=m=false;}
  void getMotorCurrent(float& l,float& r,float& m){l=r=m=0;}
};
extern TestMotorDriver motorDriver;
struct TestRobotDriver { void run(){} };
extern TestRobotDriver robotDriver;
void watchdogReset();

#include "types.h"
enum { LOC_GPS=0, LOC_REFLECTOR_STRIPS=5 };
struct TestStateEstimator { OperationType stateOp=OP_IDLE; int stateLocalizationMode=LOC_GPS; };
struct TestLineTracker { bool dockSlipTurning()const{return false;} };
extern TestLineTracker lineTracker;
extern TestStateEstimator stateEstimator;
struct TestMaps { bool undocking=false; bool isUndocking(){return undocking;} };
extern TestMaps maps;
