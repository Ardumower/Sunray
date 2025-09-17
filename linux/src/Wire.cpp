/*
  I2C Master library for Raspberry Pi Arduino.
  Copyright (c) 2015 by Hristo Gochkov

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>

  #include <fcntl.h>
  #include <unistd.h>
  #include <errno.h>
  #include <sys/select.h>
  #include <sys/stat.h>
  #include <sys/ioctl.h>
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
}

#include "Arduino.h"
#include "Wire.h"

// https://www.kernel.org/doc/Documentation/i2c/dev-interface
// https://github.com/itead/Segnix/blob/master/lib/c/itead_wire.c
// https://github.com/Digilent/linux-userspace-examples/blob/master/i2c_example_linux/src/i2c.c
// https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPiI2C.c

uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire::rxBufferIndex = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress = 0;
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire::txBufferIndex = 0;
uint8_t TwoWire::txBufferLength = 0;

uint8_t TwoWire::busAddress = 0;
int TwoWire::busFd = -1;

bool WireDebug = false;


TwoWire::TwoWire(){}

void TwoWire::begin(){  
  begin(1);
}

void TwoWire::end(void){
  //pinMode(2, INPUT);
  //pinMode(3, INPUT);
}

void TwoWire::setClock(uint32_t frequency){
  //BSC1DIV = BSCF2DIV(frequency);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop){
  if (busFd < 0) {
		perror("requestFrom error: no such I2C bus");
    return 4; // other error;
  }
  if (txBufferLength != 0){
    ::printf("WARN: I2C misorder - requestFrom in between transmission!");
  }
  if(quantity > BUFFER_LENGTH) quantity = BUFFER_LENGTH;
  if (ioctl(busFd, I2C_SLAVE, address) < 0) {
		perror("error: I2C ioctl failed");
    return 0;
  }
  uint8_t ret = ::read(busFd, rxBuffer, quantity);
  if (WireDebug) ::printf("TwoWire read addr=%x, len=%d ret=%d\n", address, quantity, quantity);  
  //uint8_t ret = twi_readFrom(address, rxBuffer, quantity, sendStop);
  rxBufferIndex = 0;
  rxBufferLength = (ret == quantity)?quantity:0;
  return rxBufferLength;
}

void TwoWire::beginTransmission(uint8_t address){
  if (WireDebug) ::printf("TwoWire beginTransmission addr=%x\n", address);    
  if (ioctl(busFd, I2C_SLAVE, address) < 0) {
		perror("error: I2C ioctl failed");
    return;
  }
  txAddress = address;
  txBufferIndex = 0;
  txBufferLength = 0;  
}

uint8_t TwoWire::endTransmission(uint8_t sendStop){  
  if (busFd < 0) {
		perror("endTransmission error: no such I2C bus");
    return 4; // other error;
  }
  uint8_t ret = ::write(busFd, txBuffer, txBufferLength);
  if (WireDebug) ::printf("TwoWire write len: %d, ret=%d\n", txBufferLength, ret);  
  //uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 0, sendStop);
  txBufferIndex = 0;
  txBufferLength = 0;
  if (ret < 0) return 4; // other error
  return 0; // success
}

uint8_t TwoWire::endTransmission(void){
  return endTransmission(true);
}

size_t TwoWire::write(uint8_t data){
  if(txBufferLength >= BUFFER_LENGTH) return 0;
  txBuffer[txBufferIndex] = data;
  ++txBufferIndex;
  txBufferLength = txBufferIndex;
  return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity){
  for(size_t i = 0; i < quantity; ++i){
    write(data[i]);
  }
  return quantity;
}

int TwoWire::available(void){
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void){
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  } else {
    if (WireDebug) ::printf("TwoWire read: nothing to read\n");
  }
  return value;
}

int TwoWire::peek(void){
  int value = -1;
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }
  return value;
}

void TwoWire::begin(uint8_t address){
  if (busFd >= 0) {
    close(busFd);
    busFd = -1;
  }
  if (WireDebug) ::printf("TwoWire begin bus=%d\n", address);  
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;
  
  //pinMode(2, ALT0);
  //pinMode(3, ALT0);
  busAddress = address; 

	// Open the given I2C bus filename.
 	char filename[50];
	sprintf(filename, "/dev/i2c-%d", busAddress);
	busFd = open(filename, O_RDWR);
	if (busFd < 0) {
		perror("error: no such I2C bus");
	}
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity){
  return requestFrom(address, quantity, (uint8_t)true);
}

TwoWire Wire = TwoWire();

