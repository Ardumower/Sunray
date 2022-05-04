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
#define SLAVE_IMU_MPU     (1 << 4)   // MPU6050
#define SLAVE_ADC         (1 << 6)   // MCP3421
#define SLAVE_IMU_BNO     (1 << 7)   // BNO055

// I/O port expander 1 channels (PCA9555) - a channel is identified by two numbers: port, pin
#define EX1_I2C_ADDR        0x21
#define EX1_IMU_POWER_PORT  1          // MT9700 powers IMU      
#define EX1_IMU_POWER_PIN   (1 << 6)   // MT9700 powers IMU

// I/O port expander 2 channels (PCA9555) - a channel is identified by two numbers: port, pin
#define EX2_I2C_ADDR        0x20
#define EX2_SWRST_PORT   1          // SWRST      
#define EX2_SWRST_PIN    (1 << 7)   // SWRST
#define EX2_NRST_PORT    1          // NRST      
#define EX2_NRST_PIN     (1 << 6)   // NRST

// ADC multiplexer channels (DG408)
#define ADC_I2C_ADDR      0x68
#define ADC_BAT1    1   // battery cell1
#define ADC_BAT2    2   // battery cell2
#define ADC_BAT3    3   // battery cell3
#define ADC_NGP_PWR 4   // ngpPWR
#define ADC_AD0     5   
#define ADC_AD1     6
#define ADC_AD1     7


// choose I2C slave via I2C multiplexer (TCA9548A)
void ioI2cMux(uint8_t addr, uint8_t slave);

// set I/O port expander (PCA9555) output
void ioExpanderOut(uint8_t addr, uint8_t port, uint8_t pin, bool level);

// read I/O port expander (PCA9555) input
bool ioExpanderIn(uint8_t addr, uint8_t port, uint8_t pin);

// choose ADC multiplexer (DG408)  
void ioAdcMux(uint8_t addr, uint8_t adc);


#ifdef __cplusplus
}
#endif

#endif

