// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "AmRobotDriver.h"
#include "../../config.h"
#include "../../helper.h"
#include "../../robot.h"
#include "../../pinman.h"


#ifndef __linux__


volatile int odomTicksLeft  = 0;
volatile int odomTicksRight = 0;

volatile unsigned long motorLeftTicksTimeout = 0;
volatile unsigned long motorRightTicksTimeout = 0;

volatile bool leftPressed = false;
volatile bool rightPressed = false;



void AmRobotDriver::begin(){
  CONSOLE.println("using robot driver: AmRobotDriver");
}



void AmRobotDriver::run(){
}


// ------------------------------------------------------------------------------------


// odometry signal change interrupt

void OdometryLeftISR(){			
  if (digitalRead(pinOdometryLeft) == LOW) return;
  if (millis() < motorLeftTicksTimeout) return; // eliminate spikes
  motorLeftTicksTimeout = millis() + 1;    
  odomTicksLeft++;    
}

void OdometryRightISR(){			
  if (digitalRead(pinOdometryRight) == LOW) return;
  if (millis() < motorRightTicksTimeout) return; // eliminate spikes
  motorRightTicksTimeout = millis() + 1;
  odomTicksRight++;  
  
  #ifdef TEST_PIN_ODOMETRY
    testValue = !testValue;
    digitalWrite(pinKeyArea2, testValue);  
  #endif
}

AmMotorDriver::AmMotorDriver(){

  // default values for all motor drivers (for parameters description, see AmRobotDriver.h)
  
  // MC33926 (https://www.nxp.com/docs/en/data-sheet/MC33926.pdf) - PwmFreqMax=20 khz
  MC33926.driverName = "MC33926";
  MC33926.forwardPwmInvert = false;
  MC33926.forwardDirLevel = LOW;
  MC33926.reversePwmInvert = true;
  MC33926.reverseDirLevel = HIGH;
  MC33926.faultActive = LOW;
  MC33926.enableActive = HIGH;
  MC33926.disableAtPwmZeroSpeed=false;  
  MC33926.keepPwmZeroSpeed = true;
  MC33926.minPwmSpeed = 0;
  MC33926.pwmFreq = PWM_FREQ_3900;
  MC33926.adcVoltToAmpOfs = 0;
  MC33926.adcVoltToAmpScale = 1.905 * 2; // ADC voltage to amp for 2 drivers connected in parallel
  MC33926.adcVoltToAmpPow = 1.0;

  // DRV8308 (https://www.ti.com/lit/ds/symlink/drv8308.pdf) - PwmFreqMax=30 khz
  DRV8308.driverName = "DRV8308";
  DRV8308.forwardPwmInvert = false;
  DRV8308.forwardDirLevel = LOW;
  DRV8308.reversePwmInvert = false;
  DRV8308.reverseDirLevel = HIGH;
  DRV8308.faultActive = LOW;
  DRV8308.enableActive = LOW;
  DRV8308.disableAtPwmZeroSpeed=false;
  DRV8308.keepPwmZeroSpeed = false; // never go to zero PWM (driver requires a periodic signal)  
  DRV8308.minPwmSpeed = 2;
  DRV8308.pwmFreq = PWM_FREQ_29300;
  DRV8308.adcVoltToAmpOfs = -1.65;
  DRV8308.adcVoltToAmpScale = 7.57; 
  DRV8308.adcVoltToAmpPow = 1.0; 

  // A4931 (https://www.allegromicro.com/en/Products/Motor-Driver-And-Interface-ICs/Brushless-DC-Motor-Drivers/~/media/Files/Datasheets/A4931-Datasheet.ashx) - PwmFreqMax=30 kHz
  A4931.driverName = "A4931";
  A4931.forwardPwmInvert = false;
  A4931.forwardDirLevel = HIGH;
  A4931.reversePwmInvert = false;
  A4931.reverseDirLevel = LOW;
  A4931.faultActive = LOW;
  A4931.enableActive = LOW;
  A4931.disableAtPwmZeroSpeed=true;
  A4931.keepPwmZeroSpeed = true;  
  A4931.minPwmSpeed = 15;    
  A4931.pwmFreq = PWM_FREQ_29300;   
  A4931.adcVoltToAmpOfs = -1.65;
  A4931.adcVoltToAmpScale = 7.57;
  A4931.adcVoltToAmpPow = 1.0; 

  // your custom brushed/brushless driver (ACT-8015A, JYQD_V7.3E3, etc.)
  CUSTOM.driverName = "CUSTOM";    // just a name for your driver
  CUSTOM.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  CUSTOM.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  CUSTOM.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  CUSTOM.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  CUSTOM.faultActive = LOW;        // fault active level (LOW or HIGH) 
  CUSTOM.enableActive = LOW;       // enable active level (LOW or HIGH)
  CUSTOM.disableAtPwmZeroSpeed=false;  // disable driver at PWM zero speed? (brake function)
  CUSTOM.keepPwmZeroSpeed = true;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  CUSTOM.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  CUSTOM.pwmFreq = PWM_FREQ_3900;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  CUSTOM.adcVoltToAmpOfs = 0;      // ADC voltage to amps (offset)
  CUSTOM.adcVoltToAmpScale = 1.00; // ADC voltage to amps (scale)
  CUSTOM.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)
}
    

void AmMotorDriver::begin(){      

  #ifdef MOTOR_DRIVER_BRUSHLESS    
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS: yes");    

    // All motors (gears, mow) are assigned individual motor drivers here.
    // NOTE: you can adjust/override default motor driver parameters here if required for a certain motor!
    // example: mowDriverChip.minPwmSpeed = 40; 

    #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308  
      mowDriverChip = DRV8308;                          
    #elif MOTOR_DRIVER_BRUSHLESS_MOW_A4931 
      mowDriverChip = A4931;
      mowDriverChip.minPwmSpeed = 40;  
    #else 
      mowDriverChip = CUSTOM;
    #endif
    
    #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308  
      gearsDriverChip = DRV8308;                         
    #elif MOTOR_DRIVER_BRUSHLESS_GEARS_A4931 
      gearsDriverChip = A4931;
    #else 
      gearsDriverChip = CUSTOM;
    #endif

  #else 
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS: no");    
    mowDriverChip = MC33926;
    gearsDriverChip = MC33926;
  #endif


  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
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
  digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
  pinMode(pinMotorMowFault, INPUT);

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  //pinMode(pinOdometryLeft2, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);
  //pinMode(pinOdometryRight2, INPUT_PULLUP);

  // lift sensor
  pinMode(pinLift, INPUT_PULLUP);

  // enable interrupts
  attachInterrupt(pinOdometryLeft, OdometryLeftISR, CHANGE);  
  attachInterrupt(pinOdometryRight, OdometryRightISR, CHANGE);  
    
	//pinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	//pinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	
}


void AmMotorDriver::run(){
}


// brushed/brushless motor driver
//(8-bit PWM=255, 10-bit PWM=1023)
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   PWM                H     Reverse

void AmMotorDriver::setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip) {
  //DEBUGLN(speed);
  if ((speed == 0) && (chip.keepPwmZeroSpeed)) {
    // driver does not require periodic signal at zero speed, we can output 'silence' for zero speed    
  } else {
    // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
    // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
    if (abs(speed) < chip.minPwmSpeed) speed = chip.minPwmSpeed * sign(speed);  
  }
  
  if (speed < 0) {    
    // reverse
    digitalWrite(pinDir, chip.reverseDirLevel) ;
    if (chip.reversePwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM

  } else {
    // forward
    digitalWrite(pinDir, chip.forwardDirLevel) ;
    if (chip.forwardPwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM
  }  
}

    
void AmMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  bool enableGears = gearsDriverChip.enableActive;
  bool enableMow = mowDriverChip.enableActive;  
  if ((leftPwm == 0) && (rightPwm == 0)){
    if (gearsDriverChip.disableAtPwmZeroSpeed){
      enableGears = !gearsDriverChip.enableActive;            
    }
  } 
  if (mowPwm == 0) {
    if (mowDriverChip.disableAtPwmZeroSpeed){
      enableMow = !mowDriverChip.enableActive;
    }
  }  
  setMotorDriver(pinMotorLeftDir, pinMotorLeftPWM, leftPwm, gearsDriverChip);
  setMotorDriver(pinMotorRightDir, pinMotorRightPWM, rightPwm, gearsDriverChip);
  setMotorDriver(pinMotorMowDir, pinMotorMowPWM, mowPwm, mowDriverChip);
  // disable driver at zero speed (brake function)
  digitalWrite(pinMotorEnable, enableGears);
  digitalWrite(pinMotorMowEnable, enableMow);
}


void AmMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){ 
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    leftFault = true;
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    rightFault = true;
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    mowFault = true;
  }
}

void AmMotorDriver::resetMotorFaults(){
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
    digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
    digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    digitalWrite(pinMotorMowEnable, !mowDriverChip.enableActive);
    digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
  }
}

void AmMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent){
  leftCurrent = pow(
      ((float)ADC2voltage(analogRead(pinMotorLeftSense))) + gearsDriverChip.adcVoltToAmpOfs, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;
  rightCurrent = pow(
      ((float)ADC2voltage(analogRead(pinMotorRightSense))) + gearsDriverChip.adcVoltToAmpOfs, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;
  mowCurrent = pow(
            ((float)ADC2voltage(analogRead(pinMotorMowSense))) + mowDriverChip.adcVoltToAmpOfs, mowDriverChip.adcVoltToAmpPow
      )  * mowDriverChip.adcVoltToAmpScale; 
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
  pinMode(pinButton, INPUT_PULLUP);  
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
  pinMode(pinRain, INPUT);
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

// ------------------------------------------------------------------------------------


void AmLiftSensorDriver::begin(){
  nextControlTime = 0;
  isLifted = false;  
}

void AmLiftSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  isLifted = (digitalRead(pinLift)== LOW);
}

bool AmLiftSensorDriver::triggered(){
  return isLifted;
}

#endif

