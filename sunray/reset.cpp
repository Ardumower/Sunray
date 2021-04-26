#include "reset.h"
#include "config.h"

#if defined(__SAMD51__)


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


#else
ResetCause getResetCause() {  
  return RST_UNKNOWN;
} 
   
#endif


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


// get free memory
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int freeMemory() {
#ifdef __linux__
  return 1000000;
#else
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
#endif
}

