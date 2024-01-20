/*
  Arduino.c -  Registers initialization for Raspberry Pi
  Copyright (c) 2015 Hristo Gochkov.  All right reserved.

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

#define ARDUINO_MAIN
#include <errno.h>
 #include <unistd.h>
#include "Arduino.h"


void sleepMicroseconds(uint32_t m){
    usleep(m);
}

void delay(uint32_t m){
    usleep(m * 1000);
}

void delayMicroseconds(uint32_t m){
    usleep(m);
}

void analogReference(uint8_t mode __attribute__((unused))){}
int analogRead(uint8_t pin __attribute__((unused))){ return 0; }

/*
 * CORE INIT AND CLOSE
 *
 * */



void uninit(){
}

/**
 * @return Return 0 on success and not 1 on failure.
 */
int init(){
    srand(time(NULL));
    return 0;
}

