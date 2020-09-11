// Adafruit Grand Central M4  (SAMD51P20A, 1024KB Flash, 256KB RAM)

// https://learn.adafruit.com/adafruit-grand-central/pinouts
// https://github.com/adafruit/ArduinoCore-samd


// FIFO size can be adjusted here (click on Arduino IDE->File->Preferences to jump into that folder):
// C:\Users\alex\AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.6.0\cores\arduino\RingBuffer.h

#include "config.h"

#if defined(__SAMD51__)

#include "variant.h"
#include "WatchdogSAMD.h"  

 
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

WatchdogSAMD watchdog;


void watchdogReset(){
  watchdog.reset();
}

void watchdogEnable(uint32_t timeout){
  watchdog.enable(timeout);
}


enum ResetCause {
  RST_UNKNOWN,
  RST_POWER_ON,
  RST_EXTERNAL,
  RST_BROWN_OUT,
  RST_WATCHDOG,
  RST_SOFTWARE,
  RST_BACKUP,
};


// C:\Users\alex\AppData\Local\Arduino15\packages\arduino\tools\CMSIS-Atmel\1.2.0\CMSIS\Device\ATMEL\samd51\include\component\rstc.h
#pragma push_macro("WDT")
#undef WDT    // Required to be able to use '.bit.WDT'. Compiler wrongly replace struct field with WDT define
ResetCause getResetCause() {
  RSTC_RCAUSE_Type resetCause;
  
  resetCause.reg = REG_RSTC_RCAUSE;
  if (resetCause.bit.POR)                                   return RST_POWER_ON;
  else if (resetCause.bit.EXT)                              return RST_EXTERNAL;
  else if (resetCause.bit.BODCORE || resetCause.bit.BODVDD) return RST_BROWN_OUT;
  else if (resetCause.bit.WDT)                              return RST_WATCHDOG;
  else if (resetCause.bit.SYST || resetCause.bit.NVM)       return RST_SOFTWARE;
  else if (resetCause.bit.BACKUP)                           return RST_BACKUP;
  return RST_UNKNOWN;
}
#pragma pop_macro("WDT")

void logResetCause(){
  CONSOLE.print("RESET cause: ");
  switch (getResetCause()){
    case RST_UNKNOWN: CONSOLE.println("unknown"); break;
    case RST_POWER_ON : CONSOLE.println("power-on"); break;
    case RST_EXTERNAL : CONSOLE.println("external"); break;
    case RST_BROWN_OUT : CONSOLE.println("brown-out"); break;
    case RST_WATCHDOG : CONSOLE.println("watchdog"); break;
    case RST_SOFTWARE : CONSOLE.println("software"); break;
    case RST_BACKUP: CONSOLE.println("backup"); break;
  }
}

void logCPUHealth(){
  CONSOLE.println("CPU: ");
}


#endif   //  __SAMD51__

