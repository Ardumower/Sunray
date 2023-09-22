/*
  Alfred MCU firmware (for RM24A/18 robot mower)
  provides Sunray-compatible robot driver (serial robot driver)
  NOTE: For compiling this file on the Alfred, see README (https://github.com/Ardumower/Sunray/blob/master/alfred/README.md)
  
  * MCU1 (main): STM32F103VET 512K flash/64K SRAM
        flash size       0x80000 
        bootloader start 0x8000000
  * MCU2 (perimeter): STM32F302xB-xC/STM32F303xB-xC (Cortex-M4, 256 KB)
  * 2x A4931ET BL driver ( https://inst.eecs.berkeley.edu/~ee192/sp18/files/A4931-Datasheet.pdf )
  *   Hangzhou Ruimeng Tech MS4931  (https://datasheet.lcsc.com/lcsc/1809131539_Hangzhou-Ruimeng-Tech-MS4931_C231944.pdf)
  * TM1652 display ( https://github.com/maxint-rd/TM16xx )
  
  RM24  J1 display SWD connectors (top: peri MCU, bottom: main MCU)
    1 rst
    2 sda
    3 clk
    4 GND
    5 3.3V

  STM32 F4 discovery SWD connector (if used as ST-Link programmer) 
    1 
    2 clk
    3 GND
    4 sda
    5 rst
    6 
  
  RM24  P15 serial2 connector     Arduino
    1 TX                            RX1
    2 RX                            TX1
    3 GND                           GND
    4 5V                            5V
    
  --------------NOT USED---------------------
  J1: perimeter,  J2: main unit 

  RM24  16 pin display connector (top pcb view)
    16 J1_SWDIO                15 J1_NRST
    14 J1_SWCLK                13 J1_3.3V
    12 J2_SWRST                11 PD10/safekey
    10 PC7/home                9 PC8/start
    8  PC9/area3/softRx        7 PA8/area2
    6  PA9/area1/tx1/softTx    5 PA11/led_status
    4  J2_SWDIO                3 J2_SWCLK
    2  GND                     1 J2_3.3V

  RM24  BL motor connector
    HC   HB  HA  5v
    GND  U   v   W


  steps for compiling on a PC:
  
  1. install Arduino STM32 board libraries ('STMicroelectronics 1.9.0'):   https://github.com/stm32duino/Arduino_Core_STM32
     https://idyl.io/arduino/how-to/program-stm32-blue-pill-stm32f103c8t6/
  
  2. Using Arduino IDE:     
    a) choose board: Generic STM32F1 series
    b) part number: Generic F103VE
    c) flash this file ('rm18.ino') to bottom MCU J1 via ST-Link (SWD protocol)
     st-link v2 tool: https://github.com/rogerclarkmelbourne/Arduino_STM32/tree/master/tools/win/stlink
  
  3. flash sunray.ino with these chassis settings:    
    #define DRV_SERIAL_ROBOT  1
    #define MPU6050
    #define TICKS_PER_REVOLUTION  304  
    #define WHEEL_BASE_CM         39         // wheel-to-wheel distance (cm), 36        
    #define WHEEL_DIAMETER        205        // wheel diameter (mm), 250                 
    #define MOTOR_PID_KP     1.0    // AM 2.0
    #define MOTOR_PID_KI     0.0    // AM 0.03
    #define MOTOR_PID_KD     0.0    // AM 0.03


  protocol examples:
    request protocol version:  AT+V,0x16

*/

#include <IWatchdog.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

//#define DEBUG 1

#define VER "RM18,1.1.16"

#define pinSwdCLK          PA14
#define pinSwdSDA          PA13

#define pinRain            PA1
#define pinLift2           PA2
#define pinLift1           PA3
#define pinLift1_2         PB0
#define pinBumperX         PA4
#define pinBumperY         PA5

#define pinBatteryT        PA6
#define pinChargeV         PA7
#define pinChargeV2        PB1
#define pinChargeI         PC4
#define pinBatteryV        PC5
#define pinBatteryRX       PB6

#define pinMotorRightPWM   PE9
#define pinMotorRightBrake PB12
#define pinMotorRightDir   PB13   
#define pinMotorRightImp   PD0
#define pinMotorRightCurr  PC1

#define pinMotorLeftPWM    PE13
#define pinMotorLeftBrake  PD8
#define pinMotorLeftDir    PD9
#define pinMotorLeftImp    PD1
#define pinMotorLeftCurr   PC0

#define pinMotorMowPWM     PE11
#define pinMotorMowImp     PD3
#define pinMotorMowBrake   PB15
#define pinMotorMowDir     PB14
#define pinMotorMowFault   PE10
#define pinMotorMowCurr    PC2

#define pinOVCheck         PE14
 
#define pinKeySafe         PD10
#define pinRelay           PD11

#define pinPower           PD12
#define pinCheckPower      PD13

#define pinKeyHome         PC7
#define pinKeyStart        PC8
#define pinKeyArea3        PC9
#define pinKeyArea2        PA8
#define pinKeyArea1        PA9    // same as pinUsart1TX
#define pinKeyLED          PA10   // same as pinUsart1RX
#define pinKeyLEDStatus    PA11   
#define pinStopButtonB     PD7
#define pinStopButton      PD2

// STM32F302xB-xC/STM32F303xB-xC (Cortex-M4, 256 KB)
#define pinUsartRX3        PC11
#define pinUsartTX3        PC10

#define pinLoraReset       PC12      
#define pinLoraSCK         PB3
#define pinLoraMISO        PB4
#define pinLoraMOSI        PB5
#define pinLoraBusy        PE1

#define pinBluetoothTX     PD5
#define pinBluetoothRX     PD6

#define pinUsart1RX        PA10  // same as pinKeyLED
#define pinUsart1TX        PA9   // same as pinKeyArea1  

#define pinMpuEnable       PB7
#define pinSCL             PB8
#define pinSDA             PB9
#define pinWpEEPROM        PE0

//#define TEST_PIN_ODOMETRY 1
//#define TEST_PINS  1

volatile int odomTicksLeft  = 0;
volatile int odomTicksRight = 0;
volatile int odomTicksMow = 0;

volatile unsigned long motorLeftTicksTimeout = 0;
volatile unsigned long motorRightTicksTimeout = 0;
volatile unsigned long motorMowTicksTimeout = 0;

volatile unsigned long motorLeftTransitionTime = 0;
volatile unsigned long motorRightTransitionTime = 0;
volatile unsigned long motorMowTransitionTime = 0;

volatile float motorLeftDurationMax = 0;
volatile float motorRightDurationMax = 0;
volatile float motorMowDurationMax = 0;

volatile bool stopButton = false;
volatile int testValue = false; 


float batVoltage = 0;
float batVoltageLP = 0;
float chgVoltage = 0;
float chgCurrent = 0;
float chgCurrentLP = 0;
float mowCurr = 0;
float mowCurrLP = 0;
int mowRecoveryState = 0;
float motorLeftCurr = 0;
float motorRightCurr = 0;
float motorLeftCurrLP = 0;
float motorRightCurrLP = 0;
float batteryTemp = 0;
int leftSpeedSet = 0;
int rightSpeedSet = 0;
int mowSpeedSet = 0;
//float leftSpeedCurr = 0;
//float rightSpeedCurr = 0;
//float mowSpeedCurr = 0;
bool motorOverload = false; 
bool motorMowFault = false;
int bumperX = 0;
int bumperY = 0;
int liftLeft = 0;
int liftRight = 0;
int rain = 0;
float rainLP = 0;
bool raining = false;
bool lift = false;
float liftLeftLP = 0;
float liftRightLP = 0;
bool bumper = false;
bool ovCheck = false;
bool enableTractionBrakes = false;
bool chargerConnected = false;

String cmd;
String cmdResponse;

unsigned long motorTimeout = 0;
unsigned long motorOverloadTimeout = 0;
unsigned long nextBatTime = 0;
unsigned long nextMotorSenseTime = 0;
unsigned long stopButtonTimeout = 0;
unsigned long nextMotorControlTime = 0;
unsigned long mowBrakeStateTimeout = 0;
int mowBrakeState = 0;

// choose one UART to use for communication
//HardwareSerial mSerial(pinUsartRX3, pinUsartTX3);  // rx, tx        - UART for perimeter MCU comm
//HardwareSerial mSerial(pinUsart1RX, pinUsart1TX); // rx, tx         - UART available at display panel
SoftwareSerial mSerial(pinKeyArea3, pinKeyArea1);   // rx, tx        - SoftUART available at display panel
HardwareSerial mSerial2(pinBluetoothRX, pinBluetoothTX);  // rx, tx  - UART available at NGP PCB 

#define CONSOLE mSerial
#define CONSOLE_BAUDRATE 19200 // 9600
//#define CONSOLE_BAUDRATE 115200

#define CONSOLE2 mSerial2
#define CONSOLE2_BAUDRATE 115200


//#define SUPER_SPIKE_ELIMINATOR 1  // advanced spike elimination  (experimental, comment out to disable)


// answer Bluetooth with CRC
void cmdAnswer(String s){  
  byte crc = 0;
  for (int i=0; i < s.length(); i++) crc += s[i];
  s += F(",0x");
  if (crc <= 0xF) s += F("0");
  s += String(crc, HEX);  
  s += F("\r\n");             
  //CONSOLE.print(s);  
  cmdResponse = s;
}

void OdometryMowISR(){			
  if (digitalRead(pinMotorMowImp) == LOW) return;
  if (millis() < motorMowTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorMowTransitionTime;
    if (duration > 5) duration = 0;
    motorMowTransitionTime = millis();
    motorMowDurationMax = 0.7 * max(((float)motorMowDurationMax), ((float)duration));
    motorMowTicksTimeout = millis() + motorMowDurationMax;
  #else
    motorMowTicksTimeout = millis() + 3;
  #endif
  odomTicksMow++;
}

void OdometryLeftISR(){			  
  if (digitalRead(pinMotorLeftImp) == LOW) return;
  if (millis() < motorLeftTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorLeftTransitionTime;
    if (duration > 5) duration = 0;
    motorLeftTransitionTime = millis();
    motorLeftDurationMax = 0.7 * max(((float)motorLeftDurationMax), ((float)duration));
    motorLeftTicksTimeout = millis() + motorLeftDurationMax;
  #else
    motorLeftTicksTimeout = millis() + 3;
  #endif
  odomTicksLeft++;    
}

void OdometryRightISR(){			
  if (digitalRead(pinMotorRightImp) == LOW) return;  
  if (millis() < motorRightTicksTimeout) return; // eliminate spikes
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorRightTransitionTime;
    if (duration > 5) duration = 0;  
    motorRightTransitionTime = millis();
    motorRightDurationMax = 0.7 * max(((float)motorRightDurationMax), ((float)duration));  
    motorRightTicksTimeout = millis() + motorRightDurationMax;
  #else
    motorRightTicksTimeout = millis() + 3;
  #endif
  odomTicksRight++;        
  
  #ifdef TEST_PIN_ODOMETRY
    testValue = !testValue;
    digitalWrite(pinKeyArea2, testValue);  
  #endif
}

void stopButtonISR(){
  if (millis() < stopButtonTimeout) return;
  stopButtonTimeout = millis() + 5;
  if (digitalRead(pinStopButton) == HIGH){      
    stopButton = true;
  } else {
    stopButton = false;
  }
}

void mower(){
  if (abs(mowSpeedSet) > 0){        
    if (millis() > mowBrakeStateTimeout){      
      mowBrakeStateTimeout = millis() + 1000;      
      if (mowSpeedSet >= 0){
        digitalWrite(pinMotorMowDir, LOW);  // set mower direction forward
      } else {
        digitalWrite(pinMotorMowDir, HIGH); // set mower direction backwards
      }              
      if (mowBrakeState >= 3){       
        //digitalWrite(pinRelay, HIGH); // motor brake off            
        //analogWrite(pinMotorMowPWM, 255);
        analogWrite(pinMotorMowPWM, mowSpeedSet);  // set mower speed          
      } 
      else {      
        if (mowBrakeState % 2 == 0){ 
          digitalWrite(pinRelay, HIGH); // motor brake off            
          analogWrite(pinMotorMowPWM, 255);  
        } else {
          digitalWrite(pinRelay, LOW); // motor brake on
          analogWrite(pinMotorMowPWM, 0);            
        }
      }
      if (mowBrakeState < 20) mowBrakeState++;            
    }
  } else {
    mowBrakeStateTimeout = millis();
    mowBrakeState = 0;
    digitalWrite(pinRelay, LOW);  // motor brake on    
    analogWrite(pinMotorMowPWM, 0); // mower speed zero
  }
}

void power(bool flag){
  if (flag){
    digitalWrite(pinPower, HIGH);
  } else {
    digitalWrite(pinPower, LOW);
  }
}

// 0 = off, 255 = full speed
void motor(){
  enableTractionBrakes = false;
  if (motorOverload) {
    leftSpeedSet = 0;
    rightSpeedSet = 0;           
  } 

  // bugfix MS4931 brushless driver sending incorrectly odometry/tire speeds for very low PWM values (e.g. 5)
  if ((leftSpeedSet > 0) && (leftSpeedSet < 15)) leftSpeedSet = 15;
  if ((rightSpeedSet > 0) && (rightSpeedSet < 15)) rightSpeedSet = 15;
  if ((leftSpeedSet < 0) && (leftSpeedSet > -15)) leftSpeedSet = -15;
  if ((rightSpeedSet < 0) && (rightSpeedSet > -15)) rightSpeedSet = -15; 

  // traction brakes
  if ((leftSpeedSet == 0) && (rightSpeedSet == 0)){
    enableTractionBrakes = true; 
  } 
  
  // ----- left traction motor ------
  // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
  // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
  //if (abs(leftSpeedSet) < 2) {
  //  if (leftSpeedSet > 0) leftSpeedSet = 2;
  //    else leftSpeedSet = -2; 
  //}
  if (leftSpeedSet >= 0){
    digitalWrite(pinMotorLeftDir, HIGH);
  } else {
    digitalWrite(pinMotorLeftDir, LOW);  
  }                           
  analogWrite(pinMotorLeftPWM, 255-abs(leftSpeedSet));
  digitalWrite(pinMotorLeftBrake, !enableTractionBrakes);   // set brakes
  
  // ----- right traction motor ------
  // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
  // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
  //if (abs(rightSpeedSet) < 2) {
  //  if (rightSpeedSet > 0) rightSpeedSet = 2;
  //   else rightSpeedSet = -2; 
  //}
  if (rightSpeedSet >= 0){
    digitalWrite(pinMotorRightDir, LOW);
  } else {
    digitalWrite(pinMotorRightDir, HIGH);      
  }
  analogWrite(pinMotorRightPWM, 255-abs(rightSpeedSet));
  digitalWrite(pinMotorRightBrake, !enableTractionBrakes);  // set brakes 
}


void readSensorsHighFrequency(){
  chgVoltage = ((float)analogRead(pinChargeV))/25.0 + 3.0;
  bool connected = (chgVoltage > 7.0);
  if (chargerConnected != connected) {
    // charger connected/unconnected event
    chargerConnected = connected;  
  } 
}

void readSensors(){
  if (millis() > stopButtonTimeout + 20000){
    stopButton = false;
  } 
  // battery  voltage
  batVoltage = ((float)analogRead(pinBatteryV))/25.0 + 3.0;  
  float w = 0.99;
  batVoltageLP = w * batVoltageLP + (1.0-w) * batVoltage;
  
  batteryTemp = ((float)analogRead(pinChargeV)) / 10.0 - 50.0;
  ovCheck = digitalRead(pinOVCheck);
  motorMowFault = (digitalRead(pinMotorMowFault) == LOW);

  // rain (lift low-pass filtering)  
  rain = analogRead(pinRain);  
  w = 0.99;
  rainLP = w * rainLP + (1.0-w) * ((float)rain);
  raining = (rainLP > 50);

  // lift
  liftRight = analogRead(pinLift1);
  liftLeft = analogRead(pinLift2);
  
  // bumper (low-pass filtering)
  bumperX = analogRead(pinBumperX);
  bumperY = analogRead(pinBumperY);  
  bumper = ((abs(bumperX-300) > 150) || (abs(bumperY-300) > 150));

  // lift low-pass filtering
  w = 0.5;
  liftLeftLP = w * liftLeftLP + (1.0-w) * ((float)liftLeft);
  liftRightLP = w * liftRightLP + (1.0-w) * ((float)liftRight);
  if ((abs(liftLeft-530) < 200) || (abs(liftRight-530) < 200)){
    liftLeftLP = liftLeft;   // reset LP if lift is not triggered
    liftRightLP = liftRight;
  }
  lift = ((abs(liftLeftLP-530) > 200) && (abs(liftRightLP-530) > 200)); // calibrated on grey/orange mower
  
  
#ifdef DEBUG
  if (stopButton){
    CONSOLE.println("STOP BUTTON");
  }  
  if (lift){
    CONSOLE.println("LIFT");
  }
  if (bumper){
    CONSOLE.println("BUMPER");
  }
  if (motorMowFault){
    CONSOLE.println("MOTOR MOW FAULT");
  }
#endif
}


// read left/right gear motor current
void readMotorCurrent(){      
  // calibrated on 'black/orange' NGP robot:
  //motorLeftCurr = ((float)analogRead(pinMotorLeftCurr)) / 150.0;
  //motorRightCurr = ((float)analogRead(pinMotorRightCurr)) / 150.0;
  //mowCurr = ((float)analogRead(pinMotorMowCurr)) / 65.0;

  // calibrated on 'grey/orange' NGP robot:    
  motorLeftCurr = pow( ((float)analogRead(pinMotorLeftCurr)), 1.0/3.0) /4.0;
  motorRightCurr = pow( ((float)analogRead(pinMotorRightCurr)), 1.0/3.0) /4.0;
  mowCurr = pow( ((float)analogRead(pinMotorMowCurr)), 1.0/3.0) /2.0;
      
  // low-pass filter
  float w = 0.9;
  motorLeftCurrLP  = w * motorLeftCurrLP + (1.0-w) * motorLeftCurr;
  motorRightCurrLP = w * motorRightCurrLP + (1.0-w) * motorRightCurr;
  w = 0.99;    
  mowCurrLP = w * mowCurrLP + (1.0-w) * mowCurr;

  if ((mowCurrLP > 4.0) || (motorLeftCurrLP > 1.5) || (motorRightCurrLP > 1.5)) {      
    // too much current: turn off motors
    //motor(0, 0);
    //mower(false);
    motorOverload = true;
    motorOverloadTimeout = millis() + 2000;       
  }

  // charging current (reading charging current does not work for some reason...)
  // maximum charging current is 1.7 amps
  chgCurrent = ((float)analogRead(pinChargeI)) * 1.0;
  w = 0.99;
  chgCurrentLP = w * chgCurrentLP + (1.0-w) * chgCurrent;
}


void setup() {
  
  // power
  pinMode(pinPower, INPUT);
  pinMode(pinCheckPower, INPUT);
  //power(true);

  delay(2000);
  CONSOLE.begin(CONSOLE_BAUDRATE);
  CONSOLE2.begin(CONSOLE2_BAUDRATE);

  #ifdef TEST_PIN_ODOMETRY
    pinMode(pinKeyArea2, OUTPUT); // odometry test pin
  #endif

  #ifdef TEST_PINS
    // for testing only
    pinMode(pinKeyStart, OUTPUT);
    pinMode(pinKeyArea1, OUTPUT);
    pinMode(pinKeyArea2, OUTPUT);
    pinMode(pinKeyArea3, OUTPUT);  
    pinMode(pinKeyHome, OUTPUT);
    pinMode(pinKeyLED, OUTPUT);
    pinMode(pinKeyLEDStatus, OUTPUT);
    pinMode(pinKeySafe, OUTPUT);
  #endif
  
  pinMode(pinOVCheck, INPUT);
  pinMode(pinRain, INPUT);
  
  //battery
  pinMode(pinBatteryV, INPUT);
  pinMode(pinChargeV, INPUT);
  pinMode(pinChargeI, INPUT);

  // lift
  pinMode(pinLift1, INPUT);
  pinMode(pinLift2, INPUT);
  
  // bumper (obstacle)
  pinMode(pinBumperX, INPUT);
  pinMode(pinBumperY, INPUT);

  // mower 
  analogWriteFrequency(20000); // 8000
  
  pinMode(pinMotorMowCurr, INPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowFault, INPUT_PULLUP);
  pinMode(pinMotorMowImp, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinMotorMowImp), OdometryMowISR, CHANGE);
  pinMode(pinRelay, OUTPUT);
  pinMode(pinMotorMowBrake, OUTPUT);
  pinMode(pinMotorMowDir, OUTPUT);
  digitalWrite(pinMotorMowDir, HIGH);
  digitalWrite(pinMotorMowBrake, LOW);
  
  
  // motor left
  pinMode(pinMotorLeftCurr, INPUT);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftBrake, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftImp, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinMotorLeftImp), OdometryLeftISR, CHANGE);

  // motor right
  pinMode(pinMotorRightCurr, INPUT);
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightBrake, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightImp, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinMotorRightImp), OdometryRightISR, CHANGE);

  //pinMode(pinLoraSCK, OUTPUT);
  //pinMode(pinSCL, OUTPUT);
  //pinMode(pinLoraMOSI, OUTPUT);
  
  // emergency button:
  // The microcontroller has a built-in pull-up resistor of 50k.
  // Two emergency stop switches are connected in series, and the switch is normally closed - when the MCU detects a high level, it stops
  pinMode(pinStopButton, INPUT_PULLUP);
  pinMode(pinStopButtonB, OUTPUT);
  digitalWrite(pinStopButtonB, LOW);
  attachInterrupt(digitalPinToInterrupt(pinStopButton), stopButtonISR, CHANGE);

  IWatchdog.begin(6000000); // 6sec
}

unsigned nextInfoTime = 0;
int lps = 0;
      


// request motor 
// AT+M,20,20,1
void cmdMotor(){
  if (cmd.length()<6) return;  
  int counter = 0;
  int lastCommaIdx = 0;
  int left=0;
  int right=0;
  int mow=0;
  //bool mow=false;  
  for (int idx=0; idx < cmd.length(); idx++){
    char ch = cmd[idx];
    //mSerial.print("ch=");
    //mSerial.println(ch);
    if ((ch == ',') || (idx == cmd.length()-1)){
      int intValue = cmd.substring(lastCommaIdx+1, ch==',' ? idx : idx+1).toInt();            
      if (counter == 1){                            
          left = intValue;
      } else if (counter == 2){
          right = intValue;
      } else if (counter == 3){
          //mow = (intValue > 0);
          mow = intValue;
      } 
      counter++;
      lastCommaIdx = idx;
    }    
  }      
#ifdef DEBUG
  CONSOLE.print("left=");
  CONSOLE.print(left);
  CONSOLE.print(" right=");
  CONSOLE.print(right);
  CONSOLE.print(" mow=");  
  CONSOLE.println(mow);
#endif
  leftSpeedSet = left;
  rightSpeedSet = right;
  mowSpeedSet = mow;
  //if (mow) mowSpeedSet = 255;
  //  else mowSpeedSet = 0;
  //if (!motorOverload) {
    // non-overload state
    //motor(left, right);
    //mower(mow);
  //} else {
    // overload state
    //digitalWrite(pinMotorLeftBrake, LOW);  // enable brakes
    //digitalWrite(pinMotorRightBrake, LOW);  // enable brakes    
  //}
  motorTimeout = millis() + 3000;
  String s = F("M");
  s += ",";
  s += odomTicksLeft;
  s += ",";
  s += odomTicksRight;
  s += ",";
  s += odomTicksMow;
  s += ",";
  s += chgVoltage;
  s += ",";
  s += int(bumper);
  s += ",";
  s += int(lift);
  s += ",";
  s += int(stopButton);
  cmdAnswer(s);
}

// perform hang test (watchdog should trigger)
void cmdTriggerWatchdog(){
  String s = F("Y");
  cmdAnswer(s);  
  while(true); // never returns
}

// request version
void cmdVersion(){
  String s = F("V,");
  s += F(VER);
  cmdAnswer(s);
}


// request summary
void cmdSummary(){
  String s = F("S,");
  s += batVoltage;  
  s += ",";
  s += chgVoltage;
  s += ",";
  s += chgCurrentLP;
  s += ",";
  s += int(lift);
  s += ",";
  s += int(bumper);
  s += ",";
  s += int(raining);
  s += ",";
  //s += int(motorMowFault);
  s += int(motorOverload);  
  s += ",";
  s += mowCurrLP;
  s += ",";
  s += motorLeftCurrLP;
  s += ",";
  s += motorRightCurrLP;
  s += ",";
  s += batteryTemp;
  cmdAnswer(s);  
}


// process request
void processCmd(bool checkCrc){
  cmdResponse = "";      
  if (cmd.length() < 4) return;
  byte expectedCrc = 0;
  int idx = cmd.lastIndexOf(',');
  if (idx < 1){
    if (checkCrc){
#ifdef DEBUG
      CONSOLE.println("CRC ERROR");
#endif      
      return;
    }
  } else {
    for (int i=0; i < idx; i++) expectedCrc += cmd[i];  
    String s = cmd.substring(idx+1, idx+5);
    int crc = strtol(s.c_str(), NULL, 16);  
    if (expectedCrc != crc){
      if (checkCrc){
#ifdef DEBUG
        CONSOLE.print("CRC ERROR");
        CONSOLE.print(crc,HEX);
        CONSOLE.print(",");
        CONSOLE.print(expectedCrc,HEX);
        CONSOLE.println();
#endif
        return;  
      }      
    } else {
      // remove CRC
      cmd = cmd.substring(0, idx);
      //CONSOLE.println(cmd);
    }    
  }     
  if (cmd[0] != 'A') return;
  if (cmd[1] != 'T') return;
  if (cmd[2] != '+') return;
  if (cmd[3] == 'V') cmdVersion();
  if (cmd[3] == 'M') cmdMotor();
  if (cmd[3] == 'S') cmdSummary();
  if (cmd[3] == 'Y') {
    if (cmd.length() <= 4){
      cmdTriggerWatchdog();   // for developers
    } else {
      //if (cmd[4] == '2') cmdGNSSReboot();   // for developers
      //if (cmd[4] == '3') cmdSwitchOffRobot();   // for developers
    }
  }

}


// process console input
void processConsole(){
  char ch;
  if (CONSOLE.available()) {
    unsigned long timeout = millis() + 10;       
    //battery.resetIdle();  
    while ( (CONSOLE.available()) && (millis() < timeout) ){               
      ch = CONSOLE.read();          
      if ((ch == '\r') || (ch == '\n')) {        
#ifdef DEBUG        
        CONSOLE.println(cmd);
#endif      
        processCmd(true);              
        CONSOLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }
  if (CONSOLE2.available()) {
    unsigned long timeout = millis() + 10;       
    //battery.resetIdle();  
    while ( (CONSOLE2.available()) && (millis() < timeout) ){               
      ch = CONSOLE2.read();          
      if ((ch == '\r') || (ch == '\n')) {        
#ifdef DEBUG        
        CONSOLE2.println(cmd);
#endif      
        processCmd(true);              
        CONSOLE2.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }
  }     
}


void printInfo(){
  CONSOLE.print("tim=");
  CONSOLE.print(millis() / 1000);
  CONSOLE.print(" lps=");
  CONSOLE.print(lps);
  CONSOLE.print(" bat=");
  CONSOLE.print(batVoltage);
  CONSOLE.print(" V"); 
  CONSOLE.print(" chg=");
  CONSOLE.print(chgVoltage);
  CONSOLE.print(" V/");
  CONSOLE.print(chgCurrent);
  CONSOLE.print(" A");
  CONSOLE.print(" mF=");
  CONSOLE.print(motorMowFault);
  CONSOLE.print(" imp=");
  CONSOLE.print(odomTicksLeft);
  CONSOLE.print(",");
  CONSOLE.print(odomTicksRight);
  CONSOLE.print(",");
  CONSOLE.print(odomTicksMow);
  CONSOLE.print(" ");
  CONSOLE.print(" lift=");
  CONSOLE.print(liftLeft);
  CONSOLE.print(",");
  CONSOLE.print(liftRight);      
  CONSOLE.print(" bum=");
  CONSOLE.print(bumperX);
  CONSOLE.print(",");
  CONSOLE.print(bumperY);
  CONSOLE.print(" rain=");
  CONSOLE.print(rain);
  CONSOLE.print(" ov=");
  CONSOLE.print(ovCheck);
  CONSOLE.println();
}

void writePulse(int pin, int count){
  for (int i=0; i < count; i++){
    digitalWrite(pin, true); 
    delay(1) ;  
    digitalWrite(pin, false); 
    delay(1);
  }
}

void testPins(){
  writePulse(pinKeyStart, 1);
  writePulse(pinKeyArea1, 2);
  writePulse(pinKeyArea2, 3);
  writePulse(pinKeyArea3, 4);  
  writePulse(pinKeyHome, 5);
  writePulse(pinKeyLED, 6);
  writePulse(pinKeyLEDStatus, 7);
  writePulse(pinKeySafe, 8);
}


void loop() {
  
  if (millis() > nextMotorControlTime){
    nextMotorControlTime = millis() + 20;
    motor();
    mower();
    readSensorsHighFrequency();
  }

  if (millis() > motorTimeout){
    leftSpeedSet = 0;
    rightSpeedSet = 0;
    mowSpeedSet = 0;
    //motor(0,0);
    //mower(false);
  }

  processConsole();

  if (millis() > nextInfoTime){
    nextInfoTime = millis() + 1000;
    #ifdef DEBUG
      printInfo();           
    #endif
    lps = 0;
    #ifdef TEST_PINS
      testPins();
    #endif
  }
   
  if (millis() > nextBatTime){
    nextBatTime = millis() + 100;
    readSensors();
  }

  if (millis() > nextMotorSenseTime){    
    nextMotorSenseTime = millis() + 100;    
    readMotorCurrent();
  }

  if (millis() > motorOverloadTimeout){
    // allow motor to turn on again after timeout
    motorOverload = false;
  }

  /*if (counter % 2 == 0){
    digitalWrite(pinLoraSCK, HIGH);
    digitalWrite(pinLoraMOSI, HIGH);
    digitalWrite(pinSCL, HIGH);
  } else {
    digitalWrite(pinLoraSCK, LOW);
    digitalWrite(pinLoraMOSI, LOW);
    digitalWrite(pinSCL, LOW);
  }*/

  lps++;
  IWatchdog.reload();
}


