// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"


void Motor::begin() {
	pwmMax = 255;
 
  #ifdef MAX_MOW_PWM
    if (MAX_MOW_PWM <= 255) {
      pwmMaxMow = MAX_MOW_PWM;
    }
    else pwmMaxMow = 255;
  #else 
    pwmMaxMow = 255;
  #endif
  
  pwmSpeedOffset = 1.0;
  
  //ticksPerRevolution = 1060/2;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm         = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / 3.1415;    // computes encoder ticks per cm (do not change)  

  motorLeftPID.Kp       = MOTOR_PID_KP;  // 2.0;  
  motorLeftPID.Ki       = MOTOR_PID_KI;  // 0.03; 
  motorLeftPID.Kd       = MOTOR_PID_KD;  // 0.03;
  motorLeftPID.reset(); 
  motorRightPID.Kp       = motorLeftPID.Kp;
  motorRightPID.Ki       = motorLeftPID.Ki;
  motorRightPID.Kd       = motorLeftPID.Kd;
  motorRightPID.reset();		 

  robotPitch = 0;
  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  
  // apply optional custom motor direction swapping 
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    motorLeftSwapDir = !motorLeftSwapDir;
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    motorRightSwapDir = !motorRightSwapDir;
  #endif

  motorError = false;
  recoverMotorFault = false;
  recoverMotorFaultCounter = 0;
  nextRecoverMotorFaultTime = 0;
  enableMowMotor = true;
  tractionMotorsEnabled = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false; 
  
  odometryError = false;  
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;  
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;  
  motorsSenseLP = 0;

  activateLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowPWMSet = 0;
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lastControlTime = 0;
  nextSenseTime = 0;
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorMowTicks = 0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;    
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  motorLeftPWMCurrLP = 0;
  motorRightPWMCurrLP=0;   
  motorMowPWMCurrLP = 0;
  
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  motorMowRpmCurr=0;  
  motorLeftRpmLast = 0;
  motorRightRpmLast = 0;
  motorLeftRpmCurrLP = 0;
  motorRightRpmCurrLP = 0;
  motorMowRpmCurrLP = 0;
  
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;

  motorRecoveryState = false;
}

void Motor::setMowMaxPwm( int val ){
  pwmMaxMow = val;
}

void Motor::speedPWM ( int pwmLeft, int pwmRight, int pwmMow )
{
  //Correct Motor Direction
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  // ensure pwm is lower than Max
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMaxMow, max(-pwmMaxMow, pwmMow)); 
  
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow);
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
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){
   setLinearAngularSpeedTimeout = millis() + 1000;
   setLinearAngularSpeedTimeoutActive = true;
   if ((activateLinearSpeedRamp) && (useLinearRamp)) {
     linearSpeedSet = 0.9 * linearSpeedSet + 0.1 * linear;
   } else {
     linearSpeedSet = linear;
   }
   angularSpeedSet = angular;   
   float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 /2);          
   float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 /2);          
   // RPM = V / (2*PI*r) * 60
   motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
   motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
//   CONSOLE.print("setLinearAngularSpeed ");
//   CONSOLE.print(linear);
//   CONSOLE.print(",");
//   CONSOLE.print(angular); 
//   CONSOLE.print(",");
//   CONSOLE.print(lspeed);
//   CONSOLE.print(",");
//   CONSOLE.println(rspeed);
}


void Motor::enableTractionMotors(bool enable){
  if (enable == tractionMotorsEnabled) return;
  if (enable)
    CONSOLE.println("traction motors enabled");
  else 
    CONSOLE.println("traction motors disabled");
  tractionMotorsEnabled = enable;
}


void Motor::setMowState(bool switchOn){
  //CONSOLE.print("Motor::setMowState ");
  //CONSOLE.println(switchOn);
  if ((enableMowMotor) && (switchOn)){
    if (abs(motorMowPWMSet) > 0) return; // mowing motor already switch ON
    motorMowSpinUpTime = millis();
    if (toggleMowDir){
      // toggle mowing motor direction each mow motor start
      motorMowForwardSet = !motorMowForwardSet;
      if (motorMowForwardSet) motorMowPWMSet = pwmMaxMow;  
        else motorMowPWMSet = -pwmMaxMow;  
    }  else  {      
      motorMowPWMSet = pwmMaxMow;  
    }
  } else {
    motorMowPWMSet = 0;  
    motorMowPWMCurr = 0;
  }

   pwmSpeedOffset = 1.0; // reset Mow SpeedOffset
}


void Motor::stopImmediately(bool includeMowerMotor){
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;      
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0;   
  if (includeMowerMotor) {
    motorMowPWMSet = 0;
    motorMowPWMCurr = 0;    
  }
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
  // reset PID
  motorLeftPID.reset();
  motorRightPID.reset();
  // reset unread encoder ticks
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}


void Motor::run() {
  if (millis() < lastControlTime + 50) return;
  
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
    
  sense();        

  // if motor driver indicates a fault signal, try a recovery   
  // if motor driver uses too much current, try a recovery     
  // if there is some error (odometry, too low current, rpm fault), try a recovery 
  if (!recoverMotorFault) {
    bool someFault = ( (checkFault()) || (checkCurrentTooHighError()) || (checkMowRpmFault()) 
                         || (checkOdometryError()) || (checkCurrentTooLowError()) );
    if (someFault){
      stopImmediately(true);
      recoverMotorFault = true;
      nextRecoverMotorFaultTime = millis() + 1000;                  
      motorRecoveryState = true;
    } 
  } 

  // try to recover from a motor driver fault signal by resetting the motor driver fault
  // if it fails, indicate a motor error to the robot control (so it can try an obstacle avoidance)  
  if (nextRecoverMotorFaultTime != 0){
    if (millis() > nextRecoverMotorFaultTime){
      if (recoverMotorFault){
        nextRecoverMotorFaultTime = millis() + 10000;
        recoverMotorFaultCounter++;                                               
        CONSOLE.print("motor fault recover counter ");
        CONSOLE.println(recoverMotorFaultCounter);
        motorDriver.resetMotorFaults();
        recoverMotorFault = false;  
        if (recoverMotorFaultCounter >= 10){ // too many successive motor faults
          //stopImmediately();
          CONSOLE.println("ERROR: motor recovery failed");
          recoverMotorFaultCounter = 0;
          motorError = true;
        }
      } else {
        CONSOLE.println("resetting recoverMotorFaultCounter");
        recoverMotorFaultCounter = 0;
        nextRecoverMotorFaultTime = 0;
        motorRecoveryState = false;
      }        
    }
  }
  
  int ticksLeft;
  int ticksRight;
  int ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  
  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  if (motorMowPWMCurr < 0) ticksMow *= -1;
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  //CONSOLE.println(motorMowTicks);

  unsigned long currTime = millis();
  float deltaControlTimeSec =  ((float)(currTime - lastControlTime)) / 1000.0;
  lastControlTime = currTime;

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)6.0) ) / deltaControlTimeSec; // assuming 6 ticks per revolution
  float lp = 0.9; // 0.995
  motorLeftRpmCurrLP = lp * motorLeftRpmCurrLP + (1.0-lp) * motorLeftRpmCurr;
  motorRightRpmCurrLP = lp * motorRightRpmCurrLP + (1.0-lp) * motorRightRpmCurr;
  motorMowRpmCurrLP = lp * motorMowRpmCurrLP + (1.0-lp) * motorMowRpmCurr;
  
  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 2) motorLeftRpmCurr = 0;
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 2) motorRightRpmCurr = 0;
  } else motorRightTicksZero = 0;

  // speed controller
  control();    
  motorLeftRpmLast = motorLeftRpmCurr;
  motorRightRpmLast = motorRightRpmCurr;
}  


// check if motor current too high
bool Motor::checkCurrentTooHighError(){
  bool motorLeftFault = (motorLeftSense > MOTOR_FAULT_CURRENT);
  bool motorRightFault = (motorRightSense > MOTOR_FAULT_CURRENT);
  bool motorMowFault = (motorMowSense > MOW_FAULT_CURRENT);
  if (motorLeftFault || motorRightFault || motorMowFault){
    CONSOLE.print("ERROR motor current too high: ");
    CONSOLE.print("  current=");
    CONSOLE.print(motorLeftSense);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSense);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSense);
    return true;
  } 
  return false; 
}


// check if motor current too low
bool Motor::checkCurrentTooLowError(){
  //CONSOLE.print(motorRightPWMCurr);
  //CONSOLE.print(",");
  //CONSOLE.println(motorRightSenseLP);
  if  (    ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (motorMowSenseLP < MOW_TOO_LOW_CURRENT)) 
        ||  ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))  ){        
    // at least one motor is not consuming current      
    // first try reovery, then indicate a motor error to the robot control (so it can try an obstacle avoidance)    
    CONSOLE.print("ERROR: motor current too low: pwm (left,right,mow)=");
    CONSOLE.print(motorLeftPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPWMCurr);
    CONSOLE.print("  average current amps (left,right,mow)=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  }
  return false;
}


// check motor driver (signal) faults
bool Motor::checkFault() {
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  if (ENABLE_FAULT_DETECTION){    
    motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  }
  if (leftFault) {
    CONSOLE.println("Error: motor driver left signaled fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor driver right signaled fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor driver mow signaled fault");
    fault = true;
  }
  return fault;
}


// check odometry errors
bool Motor::checkOdometryError() {
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if  (   ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (abs(motorLeftRpmCurrLP) < 0.001))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (abs(motorRightRpmCurrLP) < 0.001))  )
    {               
      // odometry error
      CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
      CONSOLE.print(motorLeftRpmCurrLP);
      CONSOLE.print(",");
      CONSOLE.println(motorRightRpmCurrLP);     
      return true;        
    }
  }
  return false;
}


// check motor overload
void Motor::checkOverload(){
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseLP > MOW_OVERLOAD_CURRENT);
  if (motorLeftOverload || motorRightOverload || motorMowOverload){
    if (motorOverloadDuration == 0){
      CONSOLE.print("ERROR motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }
}


// check mow rpm fault
bool Motor::checkMowRpmFault(){
  //CONSOLE.print(motorMowPWMCurr);
  //CONSOLE.print(",");
  //CONSOLE.print(motorMowPWMCurrLP);  
  //CONSOLE.print(",");
  //CONSOLE.println(motorMowRpmCurrLP);
  if (ENABLE_RPM_FAULT_DETECTION){
    if  ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (abs(motorMowRpmCurrLP) < 10.0)) {        
      CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
      CONSOLE.print(motorMowPWMCurr);
      CONSOLE.print("  pwmLP=");      
      CONSOLE.print(motorMowPWMCurrLP);      
      CONSOLE.print("  rpmLP=");
      CONSOLE.print(motorMowRpmCurrLP);
      CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
      return true;
    }
  }  
  return false;
}

// measure motor currents
void Motor::sense(){
  if (millis() < nextSenseTime) return;
  nextSenseTime = millis() + 20;
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);
  float lp = 0.995; // 0.9
  motorRightSenseLP = lp * motorRightSenseLP + (1.0-lp) * motorRightSense;
  motorLeftSenseLP = lp * motorLeftSenseLP + (1.0-lp) * motorLeftSense;
  motorMowSenseLP = lp * motorMowSenseLP + (1.0-lp) * motorMowSense; 
  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;
  motorRightPWMCurrLP = lp * motorRightPWMCurrLP + (1.0-lp) * ((float)motorRightPWMCurr);
  motorLeftPWMCurrLP = lp * motorLeftPWMCurrLP + (1.0-lp) * ((float)motorLeftPWMCurr);
  lp = 0.99;
  motorMowPWMCurrLP = lp * motorMowPWMCurrLP + (1.0-lp) * ((float)motorMowPWMCurr); 
 
  // compute normalized current (normalized to 1g gravity)
  //float leftAcc = (motorLeftRpmCurr - motorLeftRpmLast) / deltaControlTimeSec;
  //float rightAcc = (motorRightRpmCurr - motorRightRpmLast) / deltaControlTimeSec;
  float cosPitch = cos(robotPitch); 
	float pitchfactor;
  float robotMass = 1.0;
	// left wheel friction
	if (  ((motorLeftPWMCurr >= 0) && (robotPitch <= 0)) || ((motorLeftPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch; // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch;  // increase by angle
	motorLeftSenseLPNorm = abs(motorLeftSenseLP) * robotMass * pitchfactor;  
	// right wheel friction
	if (  ((motorRightPWMCurr >= 0) && (robotPitch <= 0)) || ((motorRightPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch;  // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch; // increase by angle
  motorRightSenseLPNorm = abs(motorRightSenseLP) * robotMass * pitchfactor; 

  checkOverload();  
}


void Motor::control(){  
    
  //########################  Calculate PWM for left driving motor ############################

  motorLeftPID.TaMax = 0.1;
  motorLeftPID.x = motorLeftRpmCurr;
  motorLeftPID.w  = motorLeftRpmSet;
  motorLeftPID.y_min = -pwmMax;
  motorLeftPID.y_max = pwmMax;
  motorLeftPID.max_output = pwmMax;
  motorLeftPID.compute();
  motorLeftPWMCurr = motorLeftPWMCurr + motorLeftPID.y;
  if (motorLeftRpmSet >= 0) motorLeftPWMCurr = min( max(0, (int)motorLeftPWMCurr), pwmMax); // 0.. pwmMax
  if (motorLeftRpmSet < 0) motorLeftPWMCurr = max(-pwmMax, min(0, (int)motorLeftPWMCurr));  // -pwmMax..0

  //########################  Calculate PWM for right driving motor ############################
  
  motorRightPID.TaMax = 0.1;
  motorRightPID.x = motorRightRpmCurr;
  motorRightPID.w = motorRightRpmSet;
  motorRightPID.y_min = -pwmMax;
  motorRightPID.y_max = pwmMax;
  motorRightPID.max_output = pwmMax;
  motorRightPID.compute();
  motorRightPWMCurr = motorRightPWMCurr + motorRightPID.y;
  if (motorRightRpmSet >= 0) motorRightPWMCurr = min( max(0, (int)motorRightPWMCurr), pwmMax);  // 0.. pwmMax
  if (motorRightRpmSet < 0) motorRightPWMCurr = max(-pwmMax, min(0, (int)motorRightPWMCurr));   // -pwmMax..0  

  if ((abs(motorLeftRpmSet) < 0.01) && (motorLeftPWMCurr < 30)) motorLeftPWMCurr = 0;
  if ((abs(motorRightRpmSet) < 0.01) && (motorRightPWMCurr < 30)) motorRightPWMCurr = 0;

  //########################  Print Motor Parameter to LOG ############################
  
//  CONSOLE.print("rpm set=");
//  CONSOLE.print(tempMotorLeftRpmSet);
//  CONSOLE.print(",");
//  CONSOLE.print(tempMotorRightRpmSet);
//  CONSOLE.print("   curr=");
//  CONSOLE.print(motorLeftRpmCurr);
//  CONSOLE.print(",");
//  CONSOLE.print(motorRightRpmCurr);
//  CONSOLE.print(",");
//  CONSOLE.print("   PwmOffset=");
//  CONSOLE.println(tempPwmSpeedOffset);

  //########################  Calculate PWM for mowing motor ############################
  
  motorMowPWMCurr = 0.99 * motorMowPWMCurr + 0.01 * motorMowPWMSet;

  //########################  set PWM for all motors ############################

  if (!tractionMotorsEnabled){
    motorLeftPWMCurr = motorRightPWMCurr = 0;
  }

  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
  /*if ((motorLeftPWMCurr != 0) || (motorRightPWMCurr != 0)){
    CONSOLE.print("PID curr=");
    CONSOLE.print(motorLeftRpmCurr);
    CONSOLE.print(",");  
    CONSOLE.print(motorRightRpmCurr);
    CONSOLE.print(" set=");    
    CONSOLE.print(motorLeftRpmSet);
    CONSOLE.print(",");  
    CONSOLE.print(motorRightRpmSet);
    CONSOLE.print(" PWM:");
    CONSOLE.print(motorLeftPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurr);
    CONSOLE.print(",");
    CONSOLE.println(motorMowPWMCurr);  
  }*/
}


void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}


void Motor::test(){
  CONSOLE.println("motor test - 10 revolutions");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  int pwmLeft = 200;
  int pwmRight = 200; 
  bool slowdown = true;
  unsigned long stopTicks = ticksPerRevolution * 10;
  unsigned long nextControlTime = 0;
  while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks){
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20;
      if ((slowdown) && ((motorLeftTicks + ticksPerRevolution  > stopTicks)||(motorRightTicks + ticksPerRevolution > stopTicks))){  //Letzte halbe drehung verlangsamen
        pwmLeft = pwmRight = 20;
        slowdown = false;
      }    
      if (millis() > nextInfoTime){      
        nextInfoTime = millis() + 1000;            
        dumpOdoTicks(seconds);
        seconds++;      
      }    
      if(motorLeftTicks >= stopTicks)
      {
        pwmLeft = 0;
      }  
      if(motorRightTicks >= stopTicks)
      {
        pwmRight = 0;      
      }
      
      speedPWM(pwmLeft, pwmRight, 0);
      sense();
      //delay(50);         
      watchdogReset();     
      robotDriver.run();
    }
  }  
  speedPWM(0, 0, 0);
  CONSOLE.println("motor test done - please ignore any IMU/GPS errors");
}


void Motor::plot(){
  CONSOLE.println("motor plot (left,right,mow) - NOTE: Start Arduino IDE Tools->Serial Plotter (CTRL+SHIFT+L)");
  delay(5000);
  CONSOLE.println("pwmLeft,pwmRight,pwmMow,ticksLeft,ticksRight,ticksMow");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  motorMowTicks = 0;
  int pwmLeft = 0;
  int pwmRight = 0; 
  int pwmMow = 0;
  int cycles = 0;
  int acceleration = 1;
  bool forward = true;
  unsigned long nextPlotTime = 0;
  unsigned long stopTime = millis() + 1 * 60 * 1000;
  unsigned long nextControlTime = 0;

  while (millis() < stopTime){   // 60 seconds...
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20; 

      int ticksLeft=0;
      int ticksRight=0;
      int ticksMow=0;
      motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
      motorLeftTicks += ticksLeft;
      motorRightTicks += ticksRight;
      motorMowTicks += ticksMow;

      if (millis() > nextPlotTime){ 
        nextPlotTime = millis() + 100;
        CONSOLE.print(300+pwmLeft);
        CONSOLE.print(",");  
        CONSOLE.print(300+pwmRight);
        CONSOLE.print(",");
        CONSOLE.print(pwmMow);
        CONSOLE.print(",");        
        CONSOLE.print(300+motorLeftTicks);    
        CONSOLE.print(",");
        CONSOLE.print(300+motorRightTicks);
        CONSOLE.print(",");
        CONSOLE.print(motorMowTicks);        
        CONSOLE.println();
        motorLeftTicks = 0;
        motorRightTicks = 0;
        motorMowTicks = 0;      
      }

      speedPWM(pwmLeft, pwmRight, pwmMow);
      if (pwmLeft >= 255){
        forward = false;
        cycles++; 
      }      
      if (pwmLeft <= -255){
        forward = true;
        cycles++;               
      } 
      if ((cycles == 2) && (pwmLeft >= 0)) {
        if (acceleration == 1) acceleration = 20;
          else acceleration = 1;
        cycles = 0;
      }         
      if (forward){
        pwmLeft += acceleration;
        pwmRight += acceleration;
        pwmMow += acceleration;
      } else {
        pwmLeft -= acceleration;
        pwmRight -= acceleration;
        pwmMow -= acceleration;
      }
      pwmLeft = min(255, max(-255, pwmLeft));
      pwmRight = min(255, max(-255, pwmRight));          
      pwmMow = min(255, max(-255, pwmMow));                
    }  
    //sense();
    //delay(10);
    watchdogReset();     
    robotDriver.run(); 
  }
  speedPWM(0, 0, 0);
  CONSOLE.println("motor plot done - please ignore any IMU/GPS errors");
}
