
// https://github.com/adafruit/Adafruit_SleepyDog/blob/master/utility/WatchdogSAMD.cpp

// Be careful to use a platform-specific conditional include to only make the
// code visible for the appropriate platform.  Arduino will try to compile and
// link all .cpp files regardless of platform.
#if defined(ARDUINO_ARCH_SAMD)

#include "WatchdogSAMD.h"
#include "../../config.h"
#include <sam.h>


//#define ENABLE_STACK_SAVING 1


// Adafruit Grand Central M4: flash size 1024 kb (0x100000 bytes), flash page size 512 bytes
#define FLASH_ADDRESS (0x100000 - 512) 
//#define FLASH_ADDRESS  0x3FF80
#define SP_COUNT 32
uint32_t *spReg;
uint32_t *wdg_pointer_to_page_in_flash = (uint32_t*)FLASH_ADDRESS;
  


// https://github.com/cmaglie/FlashStorage/tree/master/src
void WatchdogSAMD::clearFlashStackDump(){
  CONSOLE.println("erasing flash stack dump...");  
  uint32_t *write_pointer_to_page_in_flash = (uint32_t*)FLASH_ADDRESS;
  // erase flash page
  NVMCTRL->ADDR.reg = ((uint32_t)write_pointer_to_page_in_flash);
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EB;
  while (!NVMCTRL->INTFLAG.bit.DONE) { }  
  /*for (int i = 0; i < SP_COUNT; i++) {        
    *write_pointer_to_page_in_flash = 0xFFFFFFFF;        
    write_pointer_to_page_in_flash++;            
  }  
  //write page buffer to flash:    
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
  while (NVMCTRL->INTFLAG.bit.DONE == 0) { }  */
}


bool WatchdogSAMD::readFlashStackDump(){  
  CONSOLE.print("reading flash stack dump from last watchdog reset... sp=");
  uint32_t *read_pointer_to_page_in_flash = (uint32_t*)FLASH_ADDRESS;      
  uint32_t sp = *read_pointer_to_page_in_flash; // first address is stack pointer value at watchdog time
  read_pointer_to_page_in_flash++;            
  CONSOLE.println(sp, HEX);
  bool stackEmpty = true;
  for (int i = 0; i < SP_COUNT; i++) {        
    uint32_t sp = *read_pointer_to_page_in_flash;
    if (sp != 0xFFFFFFFF) stackEmpty = false;
    CONSOLE.print("0x");
    CONSOLE.print(sp, HEX);
    CONSOLE.print(",");
    read_pointer_to_page_in_flash++;            
  } 
  CONSOLE.println();    
  return stackEmpty;
}


void WatchdogSAMD::logFlashStackDump(){    
  if (readFlashStackDump()){
    CONSOLE.println("stack dump is empty (this does not look like a stack dump)");
  } else {
    CONSOLE.println("this looks like a real stack dump");
    CONSOLE.println("NOTE: we will need your .ino.elf binary file for further inspections");    
    clearFlashStackDump();    
  } 
  CONSOLE.println();    
}


int WatchdogSAMD::enable(int maxPeriodMS, bool isForSleep) { 

  #ifdef ENABLE_STACK_SAVING
    logFlashStackDump();
  #endif
  
  // Enable the watchdog with a period up to the specified max period in
  // milliseconds.

  // Review the watchdog section from the SAMD21 datasheet section 17:
  // http://www.atmel.com/images/atmel-42181-sam-d21_datasheet.pdf

  int cycles;
  uint8_t bits;

  if (!_initialized)
    _initialize_wdt();    

  WDT->CTRLA.bit.ENABLE = 0; // Disable watchdog for config
  while (WDT->SYNCBUSY.reg)
    ;

  // You'll see some occasional conversion here compensating between
  // milliseconds (1000 Hz) and WDT clock cycles (~1024 Hz).  The low-
  // power oscillator used by the WDT ostensibly runs at 32,768 Hz with
  // a 1:32 prescale, thus 1024 Hz, though probably not super precise.

  if ((maxPeriodMS >= 16000) || !maxPeriodMS) {
    cycles = 16384;
    bits = 0xB;
  } else {
    cycles = (maxPeriodMS * 1024L + 500) / 1000; // ms -> WDT cycles
    if (cycles >= 8192) {
      cycles = 8192;
      bits = 0xA;
    } else if (cycles >= 4096) {
      cycles = 4096;
      bits = 0x9;
    } else if (cycles >= 2048) {
      cycles = 2048;
      bits = 0x8;
    } else if (cycles >= 1024) {
      cycles = 1024;
      bits = 0x7;
    } else if (cycles >= 512) {
      cycles = 512;
      bits = 0x6;
    } else if (cycles >= 256) {
      cycles = 256;
      bits = 0x5;
    } else if (cycles >= 128) {
      cycles = 128;
      bits = 0x4;
    } else if (cycles >= 64) {
      cycles = 64;
      bits = 0x3;
    } else if (cycles >= 32) {
      cycles = 32;
      bits = 0x2;
    } else if (cycles >= 16) {
      cycles = 16;
      bits = 0x1;
    } else {
      cycles = 8;
      bits = 0x0;
    }
  }

  // Watchdog timer on SAMD is a slightly different animal than on AVR.
  // On AVR, the WTD timeout is configured in one register and then an
  // interrupt can optionally be enabled to handle the timeout in code
  // (as in waking from sleep) vs resetting the chip.  Easy.
  // On SAMD, when the WDT fires, that's it, the chip's getting reset.
  // Instead, it has an "early warning interrupt" with a different set
  // interval prior to the reset.  For equivalent behavior to the AVR
  // library, this requires a slightly different configuration depending
  // whether we're coming from the sleep() function (which needs the
  // interrupt), or just enable() (no interrupt, we want the chip reset
  // unless the WDT is cleared first).  In the sleep case, 'windowed'
  // mode is used in order to allow access to the longest available
  // sleep interval (about 16 sec); the WDT 'period' (when a reset
  // occurs) follows this and is always just set to the max, since the
  // interrupt will trigger first.  In the enable case, windowed mode
  // is not used, the WDT period is set and that's that.
  // The 'isForSleep' argument determines which behavior is used;
  // this isn't present in the AVR code, just here.  It defaults to
  // 'false' so existing Arduino code works as normal, while the sleep()
  // function (later in this file) explicitly passes 'true' to get the
  // alternate behavior.

  if (isForSleep) {
    WDT->INTFLAG.bit.EW = 1;        // Clear interrupt flag
    WDT->INTENSET.bit.EW = 1;       // Enable early warning interrupt
    WDT->CONFIG.bit.PER = 0xB;      // Period = max
    WDT->CONFIG.bit.WINDOW = bits;  // Set time of interrupt
    WDT->EWCTRL.bit.EWOFFSET = 0x0; // Early warning offset
    WDT->CTRLA.bit.WEN = 1;         // Enable window mode    
    while (WDT->SYNCBUSY.reg)
      ; // Sync CTRL write
  } else {
    WDT->INTFLAG.bit.EW = 1; // Clear interrupt flag
    //WDT->INTENCLR.bit.EW = 1;   // Disable early warning interrupt   
    WDT->EWCTRL.bit.EWOFFSET = bits; // Early warning offset    
    WDT->CONFIG.bit.PER = 0xB; // Set period for chip reset    
    //WDT->CONFIG.bit.PER = bits; // Set period for chip reset    
    WDT->CTRLA.bit.WEN = 0;     // Disable window mode
    WDT->INTENSET.bit.EW = 1;       // Enable early warning interrupt        
    while (WDT->SYNCBUSY.reg)
      ; // Sync CTRL write
  }
  
  reset();                   // Clear watchdog interval
  WDT->CTRLA.bit.ENABLE = 1; // Start watchdog now!
  while (WDT->SYNCBUSY.reg)
    ;

  return (cycles * 1000L + 512) / 1024; // WDT cycles -> ms
}

void WatchdogSAMD::reset() {
  // Write the watchdog clear key value (0xA5) to the watchdog
  // clear register to clear the watchdog timer and reset it.
  while (WDT->SYNCBUSY.reg)
    ;
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

uint8_t WatchdogSAMD::resetCause() {
  return RSTC->RCAUSE.reg;
}

void WatchdogSAMD::disable() {
  WDT->CTRLA.bit.ENABLE = 0;
  while (WDT->SYNCBUSY.reg)
    ;
}


void WDT_Handler(void) {
  // ISR for watchdog early warning, DO NOT RENAME!
  
#ifdef ENABLE_STACK_SAVING
  //WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;  
      
  // Typically, the stack begins at the end of SRAM, and will grow from higher to lower address values when 
  // data is stored in it. The stack pointer always points to the top of the stack.
  spReg = (uint32_t*)__get_MSP();  
  wdg_pointer_to_page_in_flash = (uint32_t*)FLASH_ADDRESS;
  *wdg_pointer_to_page_in_flash = (uint32_t)spReg;  // at first copy stack pointer address value to flash 
  wdg_pointer_to_page_in_flash++;        
  //copy 15 top values of stack to flash:
  for (int i = 0; i < SP_COUNT; i++) {        
    *wdg_pointer_to_page_in_flash = *spReg;        
    wdg_pointer_to_page_in_flash++;        
    spReg++;    
  }    
  //write page buffer to flash ( https://github.com/cmaglie/FlashStorage/tree/master/src )
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WP;
  while (NVMCTRL->INTFLAG.bit.DONE == 0) { };  
    
  //WDT->CTRLA.bit.ENABLE = 0; // Disable watchdog
  //while (WDT->SYNCBUSY.reg)
  //  ;
  //WDT->INTFLAG.bit.EW = 1; // Clear interrupt flag
#endif  
  
  WDT->CLEAR.reg = 0xFF; // value different than WDT_CLEAR_CLEAR_KEY causes reset
  while(true);  
}


int WatchdogSAMD::sleep(int maxPeriodMS) {

  int actualPeriodMS = enable(maxPeriodMS, true); // true = for sleep

  // Enable standby sleep mode (deepest sleep) and activate.
  // Insights from Atmel ASF library.
  PM->SLEEPCFG.bit.SLEEPMODE = 0x4; // Standby sleep mode
  while (PM->SLEEPCFG.bit.SLEEPMODE != 0x4)
    ; // Wait for it to take

  __DSB(); // Data sync to ensure outgoing memory accesses complete
  __WFI(); // Wait for interrupt (places device in sleep mode)

  // Code resumes here on wake (WDT early warning interrupt).
  // Bug: the return value assumes the WDT has run its course;
  // incorrect if the device woke due to an external interrupt.
  // Without an external RTC there's no way to provide a correct
  // sleep period in the latter case...but at the very least,
  // might indicate said condition occurred by returning 0 instead
  // (assuming we can pin down which interrupt caused the wake).

  return actualPeriodMS;
}

void WatchdogSAMD::_initialize_wdt() {
  // One-time initialization of watchdog timer.
  // Insights from rickrlh and rbrucemtl in Arduino forum!

  // SAMD51 WDT uses OSCULP32k as input clock now
  // section: 20.5.3
  OSC32KCTRL->OSCULP32K.bit.EN1K = 1;  // Enable out 1K (for WDT)
  OSC32KCTRL->OSCULP32K.bit.EN32K = 0; // Disable out 32K

  // Enable WDT early-warning interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0); // Top priority
  NVIC_EnableIRQ(WDT_IRQn);

  while (WDT->SYNCBUSY.reg)
    ;

  USB->DEVICE.CTRLA.bit.ENABLE = 0; // Disable the USB peripheral
  while (USB->DEVICE.SYNCBUSY.bit.ENABLE)
    ;                                 // Wait for synchronization
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 0; // Deactivate run on standby
  USB->DEVICE.CTRLA.bit.ENABLE = 1;   // Enable the USB peripheral
  while (USB->DEVICE.SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization

  _initialized = true;
}

#endif // defined(ARDUINO_ARCH_SAMD)

