/*
 * io.h
 *
 *  Created on: 24.11.2016
 *      Author: student
 */

#ifndef IO_H_
#define IO_H_

#include <msp430f5224.h>
#include "driverlib/MSP430F5xx_6xx/driverlib.h"
#include "ads1292.h"
#include "max30102.h"
#include "as3955.h"


//Bitmasken der GPIO Pins
#define Power_sel	BIT6	//P4.6
#define ECG_Npwdn	BIT0	//P5.0
#define ECG_start	BIT1	//P5.1
#define DebugLED	BIT2	//P5.2
#define PPG_en		BIT3	//P5.3
#define NFC_irq		BIT0	//P6.0
#define NFC_ss		BIT1	//P6.1
#define PPG_int		BIT2	//P6.2
#define Electr_det	BIT3	//P6.3
#define ECG_Ndrdy	BIT4	//P6.4
#define ECG_Ncs		BIT5	//P6.5

//Bitmasken der SPI Pins
#define SCK		BIT7	//P2.7
#define SIMO	BIT3	//P3.3
#define SOMI	BIT4	//P3.4

//Get Switch Position on Debugging Board ext->1, nfc->0
#define EXT_SWITCH GPIO_getInputPinValue(GPIO_PORT_P4, BIT6)

//Global Flags
uint8_t debugDevice;
uint8_t startDebug_Flag;
uint8_t enableDebug_Flag;
uint8_t liveMode_Flag;

uint8_t debug_led_state;

//Global NFC Variables
uint8_t nfc_data[12];
uint16_t nfc_samplecounter;

//General Functions
void clock_init();
void gpio_init();
void timer_init();
void debug_led(uint8_t b);
void debug_idle();
void ldo1v8_en(uint8_t b);
void ecg_en(uint8_t b);
void ecg_start(uint8_t b);
uint8_t ext_switch();

//UART Functions
void uart_init();
void uart_sendc(uint8_t c);
void uart_sends(char* s);

//SPI Functions
void spi_init(uint32_t spiclk);
void spi_write(uint8_t byte);
void spi_send2b(uint8_t b1, uint8_t b2);
void spi_send3b(uint8_t b1, uint8_t b2, uint8_t b3);
uint8_t spi_read(uint8_t reg);
void spi_read_continous(uint8_t *dataBuffer1, uint8_t *dataBuffer2, uint8_t *statusBuffer);

//I2C Functions
void i2c_init(uint8_t slave_address);
void i2c_write(uint8_t reg, uint8_t data);
uint8_t i2c_read(uint8_t reg);
uint8_t i2c_read_fifo();

#endif /* IO_H_ */
