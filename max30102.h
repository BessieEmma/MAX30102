/*
 * max30102.h
 *
 *  Created on: 01.12.2016
 *      Author: student
 */

#ifndef MAX30102_H_
#define MAX30102_H_

#include <msp430f5224.h>
#include "driverlib/MSP430F5xx_6xx/driverlib.h"

#include "io.h"

//MAX30102 I2C Slave Adresse
#define MAX30102	0b1010111

//MAX30102 Debugging Indicator for active Chip
#define MAX30102DEBUG	0x01

// Status Addresses
#define INTERRUPT_STATUS1_ADDR	0x00
#define INTERRUPT_STATUS2_ADDR	0x01
#define INTERRUPT_ENABLE1_ADDR	0x02
#define FIFO_WR_PTR_ADDR 		0x04
#define FIFO_OVF_COUNTER_ADDR 	0x05
#define FIFO_RD_PTR_ADDR 		0x06
#define FIFO_DATA_ADDR 			0x07

//Configuration Addresses
#define FIFO_CONFIGURATION_ADDR			0x08
#define MODE_CONFIGURATION_ADDR 		0x09
#define SPO2_CONFIGURATION_ADDR			0x0A
#define LED1_ADDR 						0x0C
#define LED2_ADDR 						0x0D
#define PILOT_PA_ADDR 					0x10
#define MULTI_LED_MODE12_ADDR 			0x11
#define MULTI_LED_MODE34_ADDR 			0x12
#define TEMPERATUR_CONFIGURATION_ADDR 	0x21
#define PROX_INT_THRESH_ADDR 			0x30

//LED Values
#define PART_ID_ADDR 0xFF
#define LED_0mA2 	0x01
#define LED_0mA4 	0x02
#define LED_1mA 	0x05
#define LED_3mA1	0x0F
#define LED_6mA4 	0x1F
#define LED_12mA5 	0x3F
#define LED_25mA4 	0x7F
#define LED_50mA 	0xFF



//Register Values
#define HEART_RATE_MODE 	0x02
#define MULTI_LED_MODE 		0x07
#define MULTI_LED_CONTROL_1 0x11
#define MULTI_LED_CONTROL_2 0x12
#define SPO2_MODE 			0x03
#define IR_LED_SLOT1 		0x02
#define ID_REG 				0xFF
#define LED_RED 			0x0C
#define LED_IR 				0x0D
#define FIFO_CONFIGURATION 	0x08
#define SMP_AVG_1 			0x00
#define SMP_AVG_2 			0x20
#define SMP_AVG_4 			0x40
#define SMP_AVG_8 			0x60
#define SMP_AVG_16 			0x80
#define SMP_AVG_32 			0xE0
#define FIFO_ROLLOVER_EN 	0x10
#define FIFO_A_FULL_15 		0x0F
#define FIFO_A_FULL_4 		0x04
#define SPO2_CONFIGURATION 	0x0A
#define SMP_RATE_800 		0x10
#define PULSE_WIDTH_118 	0x01
#define INTERRUPT_ENABLE1 	0x02
#define PPG_RDY_EN 			0x40
#define A_FULL_EN 			0x80
#define PROX_INT_EN 		0x10
#define PROX_INT_TRESH 		0x30
#define PROX_TRESH_HALF 	0x7F
#define PILOT_PA 			0x10
#define ADC_LSB_3125		0x40
#define SMP_RATE_100 		0x04
#define PULSE_WIDTH_411us 	0x03

//Samplerates with 118us Pulsewidth
#define MAX_SMP_RATE_50 	0x01
#define MAX_SMP_RATE_100 	0x05
#define MAX_SMP_RATE_200 	0x09
#define MAX_SMP_RATE_400 	0x0D
#define MAX_SMP_RATE_800 	0x11
#define MAX_SMP_RATE_1k 	0x15
#define MAX_SMP_RATE_1k6 	0x19
#define MAX_SMP_RATE_3k2 	0x1D

//FIFO Pointers
#define FIFO_WR_PTR 0x04
#define FIFO_RD_PTR 0x06

//Print Flags
#define PRINT_EN 	0x01
#define PRINT_DIS 	0x00

//Set Buffersize for FIFO
#define BUFFERSIZE 17

//Global Variables to speed up Code
uint32_t led11Buffer[BUFFERSIZE];
uint32_t led12Buffer[BUFFERSIZE];
uint32_t led13Buffer[BUFFERSIZE];
uint8_t wr_ptr, rd_ptr, samples2read, rd_ptr_new;


//General Fuctions
uint8_t max30102_init();
uint8_t max30102_int();
uint8_t max30102_debugMode();

//Read Functions
void max30102_read(int32_t *ledBuffer, uint8_t printFlag);
void max30102_readcontinous(int32_t *ledBuffer, uint8_t printFlag);
void max30102_readDebug();

#endif /* MAX30102_H_ */
