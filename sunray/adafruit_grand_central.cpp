// Adafruit Grand Central M4 

// https://learn.adafruit.com/adafruit-grand-central/pinouts

#include "config.h"

#ifndef HAVE_DUE

#include "variant.h"

 
Uart Serial2(&sercom4, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
void SERCOM4_0_Handler() { Serial2.IrqHandler(); }
void SERCOM4_1_Handler() { Serial2.IrqHandler(); }
void SERCOM4_2_Handler() { Serial2.IrqHandler(); }
void SERCOM4_3_Handler() { Serial2.IrqHandler(); }

Uart Serial3(&sercom1, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);
void SERCOM1_0_Handler() { Serial3.IrqHandler(); }
void SERCOM1_1_Handler() { Serial3.IrqHandler(); }
void SERCOM1_2_Handler() { Serial3.IrqHandler(); }
void SERCOM1_3_Handler() { Serial3.IrqHandler(); }

Uart Serial4(&sercom5, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX);
void SERCOM5_0_Handler() { Serial4.IrqHandler(); }
void SERCOM5_1_Handler() { Serial4.IrqHandler(); }
void SERCOM5_2_Handler() { Serial4.IrqHandler(); }
void SERCOM5_3_Handler() { Serial4.IrqHandler(); }


void watchdogReset(){
  // TODO
}

void watchdogEnable(uint32_t timeout){
  // TODO
}

#endif


