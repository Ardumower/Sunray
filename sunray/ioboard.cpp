#include "ioboard.h"
#include <Wire.h>
#include "config.h"


// choose I2C slave via I2C multiplexer (TCA9548A)
// slaves: bitmask of slave numbers
void ioI2cMux(uint8_t addr, uint8_t slaves){
  Wire.beginTransmission(addr); // TCA9548A address 
  Wire.write(0x00);    // control register    
  Wire.requestFrom(addr, 1);
  uint8_t state = Wire.read();   // get current control register state
  Wire.endTransmission();

  Wire.beginTransmission(addr); // TCA9548A address  
  Wire.write(state | slaves);  // enable additional I2C devices 
  Wire.endTransmission();
}

// set I/O port expander (PCA9555) output
void ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level){
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  Wire.requestFrom(addr, 1);  
  uint8_t state = Wire.read();   // get current configuration port
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state & (~pin) ); // enable pin as output 
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
    Wire.write( state | (pin) );    
  else 
    Wire.write( state & (~pin) );  
  Wire.endTransmission();
}

// read I/O port expander (PCA9555) input
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin){
  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port);    // configuration port    
  Wire.requestFrom(addr, 1);  
  uint8_t state = Wire.read();   // get current configuration port
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(6+port); // configuration port     
  Wire.write( state | (pin) ); // enable pin as input 
  Wire.endTransmission();

  Wire.beginTransmission(addr); // PCA9555 address 
  Wire.write(0+port);    // input port    
  Wire.requestFrom(addr, 1);  
  state = Wire.read();   // get current output port
  Wire.endTransmission();
  return (state & pin); 
}


// choose ADC multiplexer (DG408)  
void ioAdcMux(uint8_t addr, uint8_t adc){

}


