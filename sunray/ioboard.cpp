#include "ioboard.h"
#include <Wire.h>
#include "config.h"


// choose I2C slave via I2C multiplexer (TCA9548A)
// slave: slave number (0-7)
// enable: true or false
void ioI2cMux(uint8_t addr, uint8_t slave, bool enable){
  byte mask = (1 << slave);
  Wire.beginTransmission(addr); // TCA9548A address 
  Wire.write(0x00);    // control register    
  Wire.requestFrom(addr, 1);
  uint8_t state = Wire.read();   // get current control register state
  Wire.endTransmission();

  Wire.beginTransmission(addr); // TCA9548A address  
  if (enable)
    Wire.write(state | mask);  // enable I2C device 
  else 
    Wire.write(state & (~mask) );  // disable I2C device   
  Wire.endTransmission();
}

// set I/O port expander (PCA9555) output
// port: 0-7
// pin: 0-7
// level: true or false
void ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level){
  byte mask = (1 << pin);
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  Wire.requestFrom(addr, 1);  
  uint8_t state = Wire.read();   // get current configuration port
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state & (~mask) ); // enable pin as output 
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(2+port);    // output port    
  Wire.requestFrom(addr, 1);  
  state = Wire.read();   // get current output port
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(2+port); // output port     
  // set additional pins to desired level
  if (level)
    Wire.write( state | (mask) );    
  else 
    Wire.write( state & (~mask) );  
  Wire.endTransmission();
}

// read I/O port expander (PCA9555) input
// port: 0-7
// pin: 0-7
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin){
  byte mask = (1 << pin);
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  Wire.requestFrom(addr, 1);  
  uint8_t state = Wire.read();   // get current configuration port
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state | (mask) ); // enable pin as input 
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(0+port);    // input port    
  Wire.requestFrom(addr, 1);  
  state = Wire.read();   // get current output port
  Wire.endTransmission();
  return ((state & mask) != 0); 
}


// choose ADC multiplexer (DG408) channel  
// adc: 1-8
void ioAdcMux(uint8_t adc){
  adc = adc - 1;
  ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_A0_PORT, EX1_ADC_A0_PIN, (adc & 1) != 0);
  ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_A1_PORT, EX1_ADC_A1_PIN, (adc & 2) != 0);
  ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_A2_PORT, EX1_ADC_A2_PIN, (adc & 4) != 0);
  ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_EN_PORT, EX1_ADC_EN_PIN, true);
}

// ADC conversion (MCP3421)

// sr= Sample Rate Selection 
// sr=0  ; 00 = 240 SPS (12 bits),
// sr=1  ; 01 = 60 SPS (14 bits),
// sr=2  ; 10 = 15 SPS (16 bits),
// sr=3  ; 11 = 3.75 SPS (18 bits)
#define ADC_SR 3

// pga=  PGA Gain Selector 
// 0 = 1 V/V,
// 1 = 2 V/V,
// 2 = 4 V/V,
// 3 = 8 V/V
#define ADC_PGA 0


float ioAdc(uint8_t addr){ 
  byte sr=ADC_SR & 3;      // Sample Rate
  byte pga=ADC_PGA & 3;    // PGA ampification Factor

  // send config
  byte conf=0;    
  conf = conf | ( sr  << 2 );     
  conf = conf | pga;     
  bitWrite (conf ,7,1);   // RDY    
  bitWrite (conf ,4,1);   // O/C 1       

  Wire.beginTransmission(addr); // MCP3421 address   
  Wire.write(conf);   // config register %1000 1000
  // /RDY = 1, One Conversion, 15 samples per, PGA = X1
  // Serial.println(conf,BIN);     
  Wire.endTransmission();

  // do conversion
  long l1;
  byte b2, b3, b4;
  float db1; 
  if (sr < 3) {
    Wire.requestFrom(addr, 3);
    b2 = Wire.read();
    b3 = Wire.read();
    conf = Wire.read();
    Wire.endTransmission();
    l1= 256 * b2 + b3;
  } 
  else {
    Wire.requestFrom(addr, 4);
    b2 = Wire.read();
    b3 = Wire.read();
    b4 = Wire.read();
    conf = Wire.read();
    Wire.endTransmission();
    // _l1=_b4;    
    l1 = (long)b3*256;
    l1 = l1+b4;
    l1 = l1+0x10000 * b2;
    if ( b2 > 0x10 ) l1=l1 + 0xFF000000;
  }    
  float volt = l1 *  1e-3 / (1 << sr*2  ) ; // = 1mv * ADC * / sample rate factor
  volt = volt / (1 <<pga);            // divide by pga amplification
  return volt;
}
  
