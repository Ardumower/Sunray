// Adafruit Grand Central M4 


// Don't have Adafruit Grand Central M4 board? Then activate HAVE_DUE in file 'config.h' 

#ifndef HAVE_DUE

extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

extern void watchdogReset();
extern void watchdogEnable(uint32_t timeout);

#endif
