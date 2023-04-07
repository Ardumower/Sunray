#include "ioboard.h"
#include <Wire.h>
#include "config.h"


// choose I2C slave via I2C multiplexer (TCA9548A)
// slave: slave number (0-7)
// enable: true or false
void ioI2cMux(uint8_t addr, uint8_t slave, bool enable){
  byte mask = (1 << slave);
  Wire.requestFrom(addr, (uint8_t)1);
  uint8_t state = Wire.read();   // get current control register state
  //CONSOLE.print("I2cMux control=");
  //CONSOLE.println(state, BIN);
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
bool ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level){
  byte mask = (1 << pin);
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;  
  uint8_t state = Wire.read();   // get current configuration port

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state & (~mask) ); // enable pin as output 
  if (Wire.endTransmission() != 0) return false;
  
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(2+port);    // output port    
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;  
  state = Wire.read();   // get current output port

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(2+port); // output port     
  // set additional pins to desired level
  if (level)
    Wire.write( state | (mask) );    
  else 
    Wire.write( state & (~mask) );  
  if (Wire.endTransmission() != 0) return false;
  return true;
}

// read I/O port expander (PCA9555) input
// port: 0-7
// pin: 0-7
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin){
  byte mask = (1 << pin);
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;  
  uint8_t state = Wire.read();   // get current configuration port

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state | (mask) ); // enable pin as input 
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(0+port);    // input port    
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;  
  state = Wire.read();   // get current output port
  return ((state & mask) != 0); 
}


// choose ADC multiplexer (DG408) channel  
// adc: 1-8
bool ioAdcMux(uint8_t adc){
  int idx = adc - 1;
  //  ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_MUX_EN_PORT, EX1_ADC_MUX_EN_PIN, false);
  if (!ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_MUX_A0_PORT, EX1_ADC_MUX_A0_PIN, (idx & 1) != 0)) return false;
  if (!ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_MUX_A1_PORT, EX1_ADC_MUX_A1_PIN, (idx & 2) != 0)) return false;
  if (!ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_MUX_A2_PORT, EX1_ADC_MUX_A2_PIN, (idx & 4) != 0)) return false;
  if (!ioExpanderOut(EX1_I2C_ADDR, EX1_ADC_MUX_EN_PORT, EX1_ADC_MUX_EN_PIN, true)) return false;
  return true;  
}


// configure ADC MCP3421
bool ioAdcStart(uint8_t addr, bool repeatMode, bool reset){ 
  // send config  
  Config cfg;
  cfg.reg      = 0x00;
  cfg.bit.GAIN = eGain_x1;
  cfg.bit.SR   = eSR_12Bit;
  if (repeatMode)
    cfg.bit.OC = 1;  // 1=repeat, 0=single shot
  else 
    cfg.bit.OC = 0;
  Wire.beginTransmission(addr); // MCP3421 address   
  Wire.write(cfg.reg);   // config register 
  //The general call reset occurs if the second byte is
  //‘00000110’ (06h).
  if (reset) Wire.write(0x06);
  if (Wire.endTransmission() != 0) return false;
  return true;
}

// trigger a single ADC conversion (MCP3421)
bool ioAdcTrigger(uint8_t addr){
  Config cfg;
  cfg.reg      = 0x00;
  cfg.bit.GAIN = eGain_x1;
  cfg.bit.SR   = eSR_12Bit;
  cfg.bit.OC   = 0; // 1=repeat, 0=single shot  
  cfg.bit.RDY = 1;  // trigger conversion
  Wire.beginTransmission(addr); // MCP3421 address   
  Wire.write(cfg.reg);   // config register 
  if (Wire.endTransmission() != 0) return false;
  return true;
}


// read ADC conversion (MCP3421)
float ioAdc(uint8_t addr){

  uint8_t u8Data;
  uint8_t u8Len = 3;

  //unsigned long startTime = millis();                
  if ((u8Len != Wire.requestFrom(addr, u8Len)) ||
      (u8Len < 3)){
    CONSOLE.println("ioAdc no data");
    return -1;
  }

  u8Data     = (uint8_t)Wire.read();
  int32_t s32Value = ((u8Data & 0x80) != 0) ? -1 : 0;
  s32Value = (s32Value & 0xFFFFFF00) | u8Data;

  for (u8Len--; u8Len > 1; u8Len--)
  {
    s32Value <<= 8;
    s32Value  |= (uint8_t)Wire.read();
  }

  Config cfg;
  cfg.reg = Wire.read();
  
  //unsigned long duration = millis() - startTime;
  //CONSOLE.print("duration ");
  //CONSOLE.println(duration);  

  if (cfg.bit.RDY == 1) {    
    CONSOLE.print("ioAdc not ready - config=");
    CONSOLE.println(cfg.reg, BIN);  
    return -1;
  }    
  return ((float)s32Value) / 2048.0 * 2.048;
}
  

// read eeprom byte (BL24C256A)
byte ioEepromReadByte( uint8_t addr, unsigned int eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(addr);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  if (Wire.endTransmission() != 0) return rdata;
  if (Wire.requestFrom(addr,(uint8_t)1) != 1) return rdata;
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// write eeprom byte (BL24C256A)
bool ioEepromWriteByte( uint8_t addr, unsigned int eeaddress, byte data ) {
  Wire.beginTransmission(addr);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  if (Wire.endTransmission() != 0) return false;
  return true;
}

// write eeprom page (BL24C256A)
bool ioEepromWritePage( uint8_t addr, unsigned int eeaddresspage, byte* data, byte length ) {
  Wire.beginTransmission(addr);
  Wire.write((int)(eeaddresspage >> 8)); // MSB
  Wire.write((int)(eeaddresspage & 0xFF)); // LSB
  byte c;
  for ( c = 0; c < length; c++)
    Wire.write(data[c]);
  if (Wire.endTransmission() != 0) return false;
  return true;
}


