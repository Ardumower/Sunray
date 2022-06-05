// Ardumower Sunray 

// I/O board (Alfred)

#ifndef IOBOARD_H
#define IOBOARD_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"{
#endif

// I2C multiplexer slaves (TCA9548A)  https://www.ti.com/lit/ds/symlink/tca9548a.pdf
#define MUX_I2C_ADDR    0x70
#define SLAVE_BUS0        0   // MPU6050 (Alfred dev PCB without buzzer)
#define SLAVE_BUS1        1   
#define SLAVE_BUS2        2   
#define SLAVE_BUS3        3   
#define SLAVE_IMU_MPU     4   // MPU6050 (Alfred dev PCB with buzzer)
#define SLAVE_ADC         6   // MCP3421
#define SLAVE_IMU_BNO     7   // BNO055
#define SLAVE_EEPROM      5   // BL24C256A

// EEPROM (BL24C256A)  https://datasheet.lcsc.com/lcsc/1810111121_BL-Shanghai-Belling-BL24C256A-PARC_C90485.pdf
#define EEPROM_I2C_ADDR     0x50  

// I/O port expander 1 channels (PCA9555)   https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
// a channel is identified by two numbers: port, pin
// (IMU power, fan power, adc mux)
#define EX1_I2C_ADDR        0x21
#define EX1_IMU_POWER_PORT  1        // IO1.6  MT9700 powers IMU      
#define EX1_IMU_POWER_PIN   6        // IO1.6  MT9700 powers IMU
#define EX1_FAN_POWER_PORT  1        // IO1.7  MT9700 powers fan      
#define EX1_FAN_POWER_PIN   7        // IO1.7  MT9700 powers fan
#define EX1_ADC_MUX_A0_PORT     1                
#define EX1_ADC_MUX_A0_PIN      0   
#define EX1_ADC_MUX_A1_PORT     1                
#define EX1_ADC_MUX_A1_PIN      1   
#define EX1_ADC_MUX_A2_PORT     1          
#define EX1_ADC_MUX_A2_PIN      2   
#define EX1_ADC_MUX_EN_PORT     1          
#define EX1_ADC_MUX_EN_PIN      3   

// I/O port expander 2 channels (PCA9555)  https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
// a channel is identified by two numbers: port, pin
// (buzzer, swd-prog port switching)
#define EX2_I2C_ADDR     0x20
#define EX2_BUZZER_PORT  1          // Buzzer
#define EX2_BUZZER_PIN   1          // Buzzer
#define EX2_CS6_PORT     0          // CS6  (SWD prog. port switching)    
#define EX2_CS6_PIN      6          // CS6  (SWD prog. port switching)

// I/O port expander 3 channels (PCA9555)  https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
// a channel is identified by two numbers: port, pin
// (panel LEDs)
#define EX3_I2C_ADDR     0x22
#define EX3_LED1_GREEN_PORT    0          
#define EX3_LED1_GREEN_PIN     0          
#define EX3_LED1_RED_PORT      0          
#define EX3_LED1_RED_PIN       1          
#define EX3_LED2_GREEN_PORT    0          
#define EX3_LED2_GREEN_PIN     2          
#define EX3_LED2_RED_PORT      0          
#define EX3_LED2_RED_PIN       3           
#define EX3_LED3_GREEN_PORT    0          
#define EX3_LED3_GREEN_PIN     4          
#define EX3_LED3_RED_PORT      0          
#define EX3_LED3_RED_PIN       5         

// ADC multiplexer channels (DG408)   https://www.vishay.com/docs/70062/dg408.pdf
#define ADC_BAT1     1   // battery cell1
#define ADC_BAT2     2   // battery cell2
#define ADC_BAT3     3   // battery cell3
#define ADC_MCU_ANA  4   // mcuAna
#define ADC_AD0      5   
#define ADC_AD1      6
#define ADC_AD2      7
#define ADC_BAT      8   // battery voltage

// ADC (MCP3421) config   http://ww1.microchip.com/downloads/en/devicedoc/22003e.pdf
#define ADC_I2C_ADDR   0x68

typedef union {
    struct {
    uint8_t GAIN : 2; // bit 0
    uint8_t SR   : 2; 
    uint8_t OC   : 1;
    uint8_t Cx   : 2;
    uint8_t RDY  : 1; // bit 7
    } bit;
    uint8_t reg;
} Config;

enum EGain {
    eGain_x1 = 0,
    eGain_x2,
    eGain_x4,
    eGain_x8,
};
	
enum ESampleRate {
    eSR_12Bit = 0,
    eSR_14Bit,
    eSR_16Bit,
    eSR_18Bit,
};


// choose I2C slave via I2C multiplexer (TCA9548A)
void ioI2cMux(uint8_t addr, uint8_t slave, bool enable);

// set I/O port expander (PCA9555) output
bool ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level);

// read I/O port expander (PCA9555) input
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin);

// choose ADC multiplexer (DG408) channel  
bool ioAdcMux(uint8_t adc);

// ADC start (MCP3421)
bool ioAdcStart(uint8_t addr, bool repeatMode, bool reset);

// ADC conversion (MCP3421)
bool ioAdcTrigger(uint8_t addr);
float ioAdc(uint8_t addr);

// EEPROM (BL24C256A)
bool ioEepromWriteByte( uint8_t addr, unsigned int eeaddress, byte data );
byte ioEepromReadByte( uint8_t addr, unsigned int eeaddress );
bool ioEepromWritePage( uint8_t addr, unsigned int eeaddresspage, byte* data, byte length );


#ifdef __cplusplus
}
#endif

#endif

