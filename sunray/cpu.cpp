#include "cpu.h"
#include "config.h"


#if defined(__SAMD51__)
#include <sam.h>

// MUXPOS
#define SCALEDCOREVCC   0x18  // 1/4 Scaled Core Supply
#define SCALEDVBAT      0x19  // 1/4 Scaled VBAT Supply
#define SCALEDIOVCC     0x1A  // 1/4 Scaled I/O Supply  (nominal 3.3V/4)
#define BANDGAP         0x1B  // Bandgap Voltage
#define PTAT            0x1C  // Temperature Sensor
#define CTAT            0x1D  // Temperature Sensor
// MUXNEG
#define GND             0x18  // Internal ground

#define NVMCTRL_TEMP_LOG   NVMCTRL_TEMP_LOG_W0


uint32_t readADC(uint8_t channel){
  uint32_t valueRead = 0;
 
  Adc *adc;
  adc = ADC0;
  
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
  adc->INPUTCTRL.bit.MUXPOS = channel; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  
  adc->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  adc->SWTRIG.bit.START = 1;

  // Store the value
  while (adc->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = adc->RESULT.reg;

  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  adc->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
  return valueRead;
}


// Decimal to fraction conversion. (adapted from ASF sample).
float convert_dec_to_frac(uint8_t val) {
    float float_val = (float)val;
    if (val < 10) {
        return (float_val/10.0);
    } else if (val < 100) {
        return (float_val/100.0);
    } else {
        return (float_val/1000.0);
    }
}


// https://github.com/adafruit/circuitpython/blob/master/ports/atmel-samd/common-hal/microcontroller/Processor.c

float calcTemperature(uint16_t TP, uint16_t TC) {
    uint32_t TLI = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >> FUSES_ROOM_TEMP_VAL_INT_Pos;
    uint32_t TLD = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
    float TL = TLI + convert_dec_to_frac(TLD);

    uint32_t THI = (*(uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >> FUSES_HOT_TEMP_VAL_INT_Pos;
    uint32_t THD = (*(uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >> FUSES_HOT_TEMP_VAL_DEC_Pos;
    float TH = THI + convert_dec_to_frac(THD);

    uint16_t VPL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_PTAT_ADDR & FUSES_ROOM_ADC_VAL_PTAT_Msk) >> FUSES_ROOM_ADC_VAL_PTAT_Pos;
    uint16_t VPH = (*(uint32_t *)FUSES_HOT_ADC_VAL_PTAT_ADDR & FUSES_HOT_ADC_VAL_PTAT_Msk) >> FUSES_HOT_ADC_VAL_PTAT_Pos;

    uint16_t VCL = (*(uint32_t *)FUSES_ROOM_ADC_VAL_CTAT_ADDR & FUSES_ROOM_ADC_VAL_CTAT_Msk) >> FUSES_ROOM_ADC_VAL_CTAT_Pos;
    uint16_t VCH = (*(uint32_t *)FUSES_HOT_ADC_VAL_CTAT_ADDR & FUSES_HOT_ADC_VAL_CTAT_Msk) >> FUSES_HOT_ADC_VAL_CTAT_Pos;

    // From SAMD51 datasheet: section 45.6.3.1 (page 1327).
    return (TL*VPH*TC - VPL*TH*TC - TL*VCH*TP + TH*VCL*TP) / (VCL*TP - VCH*TP - VPL*TC + VPH*TC);
}


void LogCPUHealth(){
  uint32_t valueRead;
  float voltage;
  
  CONSOLE.print("CPU: ");
  
  // ------ cpu temperature --------
  // The voltage reference output is enabled/disabled by setting/clearing the Voltage Reference Output Enable bit in the
  // Voltage Reference register (VREF.VREFOE).
  // The temperature sensor is enabled/disabled by setting/clearing the Temperature Sensor Enable bit in the Voltage
  // Reference register (VREF.TSEN).
  // Note: When VREF.ONDEMAND=0, it is not recommended to enable both voltage reference output and temperature
  // sensor at the same time - only the voltage reference output will be present at both ADC inputs.
  
  // If the SUPC is not in on-demand mode (SUPC.VREF.ONDEMAND=0), and if SUPC.VREF.TSEN=1 and
  // SUPC.VREF.VREFOE=0, the temperature sensor is selected by writing to the Temperature Sensor Channel
  // Selection bit in the Voltage Reference System Control register (SUPC.VREF.TSSEL).
  //
  // If the SUPC is in on-demand mode in (SUPC.VREF.ONDEMAND=1) and SUPC.VREF.TSEN=1, the output will
  // be automatically set to the sensor requested by the ADC, independent of SUPC.VREF.TSSEL.
  // SUPC.VREF.VREFOE can also be set to '1'.
    
  //analogReference(AR_INTERNAL1V65);     // ref 1/2 VDDANA = 1.65
  SUPC->VREF.bit.TSSEL = 0;      
  SUPC->VREF.bit.TSEN = 1;        
  uint32_t ptat = readADC(PTAT);        
  SUPC->VREF.bit.TSEN = 0;     
  
  SUPC->VREF.bit.TSSEL = 1;    
  SUPC->VREF.bit.TSEN = 1;  
  uint32_t ctat = readADC(CTAT);      
  SUPC->VREF.bit.TSEN = 0;    
    
  CONSOLE.print("PTAT=");      
  CONSOLE.print(ptat);      
  CONSOLE.print(" CTAT=");        
  CONSOLE.print(ctat);     
  
  // cpu temperatures need translation by calibrated data
  float temp = calcTemperature(ptat, ctat);
  CONSOLE.print(" deg=");     
  CONSOLE.print(temp);     
  
  // ------- cpu voltages -------
  //analogReference(AR_INTERNAL1V0);     // ref 1.0V
  CONSOLE.print(" voltages: I/O=");  
  valueRead = readADC(SCALEDIOVCC);  
  voltage  = (((float)valueRead) / 4095.0f) * 3.3f * 4.0f;    // 12 bit, ref 3.3 volt
  CONSOLE.print(voltage);                       
  
  CONSOLE.print(" Core=");
  valueRead = readADC(SCALEDCOREVCC);  
  voltage  = (((float)valueRead) / 4095.0f) * 3.3f * 4.0f;    // 12 bit, ref 3.3 volt
  CONSOLE.print(voltage);                       
  
  CONSOLE.print(" VBAT=");
  valueRead = readADC(SCALEDVBAT);  
  voltage  = (((float)valueRead) / 4095.0f) * 3.3f * 4.0f;    // 12 bit, ref 3.3 volt
  CONSOLE.print(voltage);                           
  
  //analogReference(AR_DEFAULT);  // ref default
  
}

float GetCPUTemperature(){
  // ------ cpu temperature --------
  // The voltage reference output is enabled/disabled by setting/clearing the Voltage Reference Output Enable bit in the
  // Voltage Reference register (VREF.VREFOE).
  // The temperature sensor is enabled/disabled by setting/clearing the Temperature Sensor Enable bit in the Voltage
  // Reference register (VREF.TSEN).
  // Note: When VREF.ONDEMAND=0, it is not recommended to enable both voltage reference output and temperature
  // sensor at the same time - only the voltage reference output will be present at both ADC inputs.
  
  // If the SUPC is not in on-demand mode (SUPC.VREF.ONDEMAND=0), and if SUPC.VREF.TSEN=1 and
  // SUPC.VREF.VREFOE=0, the temperature sensor is selected by writing to the Temperature Sensor Channel
  // Selection bit in the Voltage Reference System Control register (SUPC.VREF.TSSEL).
  //
  // If the SUPC is in on-demand mode in (SUPC.VREF.ONDEMAND=1) and SUPC.VREF.TSEN=1, the output will
  // be automatically set to the sensor requested by the ADC, independent of SUPC.VREF.TSSEL.
  // SUPC.VREF.VREFOE can also be set to '1'.
    
  //analogReference(AR_INTERNAL1V65);     // ref 1/2 VDDANA = 1.65
  SUPC->VREF.bit.TSSEL = 0;      
  SUPC->VREF.bit.TSEN = 1;        
  uint32_t ptat = readADC(PTAT);        
  SUPC->VREF.bit.TSEN = 0;     
  
  SUPC->VREF.bit.TSSEL = 1;    
  SUPC->VREF.bit.TSEN = 1;  
  uint32_t ctat = readADC(CTAT);      
  SUPC->VREF.bit.TSEN = 0;    
    
  //CONSOLE.print("PTAT=");      
  //CONSOLE.print(ptat);      
  //CONSOLE.print(" CTAT=");        
  //CONSOLE.print(ctat);     
  
  // cpu temperatures need translation by calibrated data
  float temp = calcTemperature(ptat, ctat);  
  //CONSOLE.print(" deg=");     
  //CONSOLE.print(temp);     
  return temp;
}

#else
  void LogCPUHealth(){}
  float GetCPUTemperature(){
    return 0;
  }

#endif


