// Ardumower Sunray 

// I/O board (Alfred)

#ifndef IOBOARD_H
#define IOBOARD_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"{
#endif

// I2C multiplexer slaves (TCA9548A)
#define MUX_I2C_ADDR    0x70
#define SLAVE_BUS0        0   // MPU6050 (Alfred dev PCB without buzzer)
#define SLAVE_IMU_MPU     4   // MPU6050 (Alfred dev PCB with buzzer)
#define SLAVE_ADC         6   // MCP3421
#define SLAVE_IMU_BNO     7   // BNO055

// I/O port expander 1 channels (PCA9555) - a channel is identified by two numbers: port, pin
#define EX1_I2C_ADDR        0x21
#define EX1_IMU_POWER_PORT  1        // MT9700 powers IMU      
#define EX1_IMU_POWER_PIN   6        // MT9700 powers IMU
#define EX1_ADC_MUX_A0_PORT     1                
#define EX1_ADC_MUX_A0_PIN      0   
#define EX1_ADC_MUX_A1_PORT     1                
#define EX1_ADC_MUX_A1_PIN      1   
#define EX1_ADC_MUX_A2_PORT     1          
#define EX1_ADC_MUX_A2_PIN      2   
#define EX1_ADC_MUX_EN_PORT     1          
#define EX1_ADC_MUX_EN_PIN      3   

// I/O port expander 2 channels (PCA9555) - a channel is identified by two numbers: port, pin
#define EX2_I2C_ADDR     0x20
#define EX2_BUZZER_PORT  1          // Buzzer
#define EX2_BUZZER_PIN   1          // Buzzer
#define EX2_SWRST_PORT   1          // SWRsT      
#define EX2_SWRST_PIN    7          // SWRsT
#define EX2_NRST_PORT    1          // NRsT      
#define EX2_NRST_PIN     6          // NRsT


// ADC multiplexer channels (DG408)
#define ADC_BAT1     1   // battery cell1
#define ADC_BAT2     2   // battery cell2
#define ADC_BAT3     3   // battery cell3
#define ADC_NGP_PWR  4   // ngpPWR
#define ADC_AD0      5   
#define ADC_AD1      6
#define ADC_AD2      7
#define ADC_BAT      8   // battery voltage

// ADC (MCP3421) config
#define ADC_I2C_ADDR   0x68

typedef union {
    struct {
    uint8_t GAIN : 2;
    uint8_t SR   : 2;
    uint8_t OC   : 1;
    uint8_t Cx   : 2;
    uint8_t RDY  : 1;
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
void ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level);

// read I/O port expander (PCA9555) input
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin);

// choose ADC multiplexer (DG408) channel  
void ioAdcMux(uint8_t adc);

// ADC start (MCP3421)
float ioAdcStart(uint8_t addr);

// ADC conversion (MCP3421)
void ioAdcTrigger(uint8_t addr);
float ioAdc(uint8_t addr);


#ifdef __cplusplus
}
#endif

#endif

