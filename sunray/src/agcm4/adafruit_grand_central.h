// Adafruit Grand Central M4  (SAMD51P20A, 1024KB Flash, 256KB RAM)


#if defined(__SAMD51__)

#ifndef ADAFRUIT_GRAND_CENTRAL
#define ADAFRUIT_GRAND_CENTRAL

extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

extern void watchdogReset();
extern void watchdogEnable(uint32_t timeout);

#endif  // ADAFRUIT_GRAND_CENTRAL

#endif  // __SAMD51__

