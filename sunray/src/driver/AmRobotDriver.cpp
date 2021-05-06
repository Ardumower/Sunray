// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "AmRobotDriver.h"
#include "../../config.h"
#include "../../helper.h"
#include "../../robot.h"


volatile int odomTicksLeft  = 0;
volatile int odomTicksRight = 0;


volatile bool leftPressed = false;
volatile bool rightPressed = false;


bool faultActive  = LOW; 
bool enableActive = LOW; 


void AmRobotDriver::begin(){
}



void AmRobotDriver::run(){
}


// ------------------------------------------------------------------------------------


// odometry signal change interrupt

void OdometryLeftISR(){			
  if (digitalRead(pinOdometryLeft) == LOW) return;  
  odomTicksLeft++;    
}

void OdometryRightISR(){			
  if (digitalRead(pinOdometryRight) == LOW) return;
  odomTicksRight++;  
}

AmMotorDriver::AmMotorDriver(){
}
    

void AmMotorDriver::begin(){
  #ifdef MOTOR_DRIVER_BRUSHLESS
    // logic for brushless drivers
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS: yes");
    faultActive  = LOW; 
    enableActive = LOW; 
  #else 
    // logic for brushed drivers    
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS: no");
    faultActive  = LOW; 
    enableActive = HIGH; 
  #endif

  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, enableActive);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorLeftFault, INPUT);

  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightSense, INPUT);
  pinMode(pinMotorRightFault, INPUT);

  // mower motor
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowSense, INPUT);
  //pinMode(pinMotorMowRpm, INPUT);
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, enableActive);
  pinMode(pinMotorMowFault, INPUT);

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  //pinMode(pinOdometryLeft2, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);
  //pinMode(pinOdometryRight2, INPUT_PULLUP);
	  
  // enable interrupts
  attachInterrupt(pinOdometryLeft, OdometryLeftISR, CHANGE);  
  attachInterrupt(pinOdometryRight, OdometryRightISR, CHANGE);  
    
	//pinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	//pinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	
}


void AmMotorDriver::run(){
}


// brushed motor driver (MC33926)
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse
void AmMotorDriver::setMC33926(int pinDir, int pinPWM, int speed) {
  //DEBUGLN(speed);
  if (speed < 0) {
    digitalWrite(pinDir, HIGH) ;
    pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)));     
  } else {
    digitalWrite(pinDir, LOW) ;
    pinMan.analogWrite(pinPWM, ((byte)speed));
  }
}


// brushless motor driver
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// PWM                H     Reverse
// verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
// Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
void AmMotorDriver::setBrushless(int pinDir, int pinPWM, int speed) {
  //DEBUGLN(speed);
  if (speed < 0) {
    digitalWrite(pinDir, HIGH) ;
    if (speed >= -2) speed = -2;                         
    pinMan.analogWrite(pinPWM, ((byte)abs(speed)));      
  } else {
    digitalWrite(pinDir, LOW) ;
    if (speed <= 2) speed = 2;                           
    pinMan.analogWrite(pinPWM, ((byte)abs(speed)));      
  }
}

    
void AmMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){
  #ifdef MOTOR_DRIVER_BRUSHLESS
    setBrushless(pinMotorLeftDir, pinMotorLeftPWM, leftPwm);
    setBrushless(pinMotorRightDir, pinMotorRightPWM, rightPwm);
    setBrushless(pinMotorMowDir, pinMotorMowPWM, mowPwm);
  #else
    setMC33926(pinMotorLeftDir, pinMotorLeftPWM, leftPwm);
    setMC33926(pinMotorRightDir, pinMotorRightPWM, rightPwm);
    setMC33926(pinMotorMowDir, pinMotorMowPWM, mowPwm);
  #endif
}


void AmMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){ 
  if (digitalRead(pinMotorLeftFault) == faultActive) {
    leftFault = true;
  }
  if  (digitalRead(pinMotorRightFault) == faultActive) {
    rightFault = true;
  }
  if (digitalRead(pinMotorMowFault) == faultActive) {
    mowFault = true;
  }
}

void AmMotorDriver::resetMotorFaults(){
  if (digitalRead(pinMotorLeftFault) == faultActive) {
    digitalWrite(pinMotorEnable, !enableActive);
    digitalWrite(pinMotorEnable, enableActive);
  }
  if  (digitalRead(pinMotorRightFault) == faultActive) {
    digitalWrite(pinMotorEnable, !enableActive);
    digitalWrite(pinMotorEnable, enableActive);
  }
  if (digitalRead(pinMotorMowFault) == faultActive) {
    digitalWrite(pinMotorMowEnable, !enableActive);
    digitalWrite(pinMotorMowEnable, enableActive);
  }
}

void AmMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent){
    float offset      = -1.65;
    #ifdef MOTOR_DRIVER_BRUSHLESS
      float scale       = 7.57;   // ADC voltage to amp      
      leftCurrent = (((float)ADC2voltage(analogRead(pinMotorRightSense))) + offset) *scale;
      rightCurrent = (((float)ADC2voltage(analogRead(pinMotorLeftSense))) + offset) *scale;
      mowCurrent = (((float)ADC2voltage(analogRead(pinMotorMowSense))) + offset) *scale; 
    #else
      float scale       = 1.905;   // ADC voltage to amp      
      leftCurrent = ((float)ADC2voltage(analogRead(pinMotorRightSense))) *scale;
      rightCurrent = ((float)ADC2voltage(analogRead(pinMotorLeftSense))) *scale;
      mowCurrent = ((float)ADC2voltage(analogRead(pinMotorMowSense))) *scale  *2;	          
    #endif
}

void AmMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = odomTicksLeft;
  rightTicks = odomTicksRight;  
  mowTicks = 0;
  // reset counters
  odomTicksLeft = odomTicksRight = 0;
}    




// ------------------------------------------------------------------------------------


// --- battery switch off circuit --------------------
// JP8 Dauer-ON : automatic switch off circuit disabled
// JP8 Autom.   : automatic switch off circuit enabled
// Note: to increase hardware switch-off time increase capacitor C12  (under DC/DC module)


void AmBatteryDriver::begin(){
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage

  //INA169:  A precision amplifier measures the voltage across the Rs=0.1 ohm, 1% sense resistor. 
  //The Rs is rated for 2W continuous so you can measure up to +5A continuous. 
  //The output is a current that is drawn through the on-board RL=10K+10K=20K resistors so that the 
  // output voltage is 2V per Amp. So for 1A draw, the output will be 2V. You can change the 
  // load resistor RL to be smaller by soldering the bridge If you solder the bridge (RL=10K resistor) 
  // you'll get 1V per Amp.   
  //
  // Is = Vout * 1k / (Rs * RL)

  // PCB1.3 (external INA module)
  //   a) bridged      RL=10K:    Is = 1V * 1k / (0.1*10K)  = 1A
  //   b) non-bridged  RL=20k:    Is = 1V * 1k / (0.1*20K)  = 0.5A
  // PCB1.4 (INA soldered on main PCB)
  //   a) bridged      RL=6.8K:   Is = 1V * 1k / (0.05*6.8K)  = 2.941A
  //   b) non-bridged  RL=10.1k:  Is = 1V * 1k / (0.05*10.1K)  = 1.98A
  
  currentFactor = CURRENT_FACTOR;         // ADC voltage to current ampere  (0.5 for non-bridged)

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);  
}


void AmBatteryDriver::run(){
}

    
float AmBatteryDriver::getBatteryVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinBatteryVoltage))) * batteryFactor;
  return voltage;  
}

float AmBatteryDriver::getChargeVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinChargeVoltage))) * batteryFactor;
  return voltage;
}


float AmBatteryDriver::getChargeCurrent(){    
  float amps = ((float)ADC2voltage(analogRead(pinChargeCurrent))) * currentFactor;    
	return amps;
}

void AmBatteryDriver::enableCharging(bool flag){
  digitalWrite(pinChargeRelay, flag);      
}

void AmBatteryDriver::keepPowerOn(bool flag){
  digitalWrite(pinBatterySwitch, flag);
}


// ------------------------------------------------------------------------------------
void BumperLeftInterruptRoutine(){
  leftPressed = (digitalRead(pinBumperLeft) == LOW);  
}

void BumperRightInterruptRoutine(){
  rightPressed = (digitalRead(pinBumperRight) == LOW);  
}


void AmBumperDriver::begin(){	
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
  attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
}

void AmBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = leftPressed;
  rightBumper = rightPressed;
}

bool AmBumperDriver::obstacle(){
  return (leftPressed || rightPressed);
}
    

void AmBumperDriver::run(){  
}


// ------------------------------------------------------------------------------------


void AmStopButtonDriver::begin(){
  nextControlTime = 0;
  pressed = false;  
}

void AmStopButtonDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  pressed = (digitalRead(pinButton)== LOW);
}

bool AmStopButtonDriver::triggered(){
  return pressed;
}


// ------------------------------------------------------------------------------------


void AmRainSensorDriver::begin(){
  nextControlTime = 0;
  isRaining = false;  
}

void AmRainSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  isRaining = (digitalRead(pinRain)== LOW);
}

bool AmRainSensorDriver::triggered(){
  return isRaining;
}

