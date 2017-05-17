/*
 * max30102.c
 *
 *  Created on: 01.12.2016
 *      Author: student
 */


#include "max30102.h"

uint8_t max30102_init()
{
	uint8_t ID = 0;
	ID = i2c_read(ID_REG);	//Wakeup
	ID = i2c_read(ID_REG);	//Read ID (default: 0x15)

	//Configure MAX30102 in Multi LED Mode with IR LED in Slot 1 with 12,5mA current
	i2c_write (MODE_CONFIGURATION_ADDR, MULTI_LED_MODE);
	i2c_write (MULTI_LED_CONTROL_1, IR_LED_SLOT1);
	i2c_write (LED_IR, LED_12mA5);

	//Configure FIFO Size dependend on selected Buffersize (default 17) with auto Rollover and no averaging
	i2c_write (FIFO_CONFIGURATION, (SMP_AVG_1 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));

	//Configure ADC LSB Size to 31,25pA, Sample Rate to 100 Smp/s and a Pulse width of 411us
	i2c_write (SPO2_CONFIGURATION, (ADC_LSB_3125 + SMP_RATE_100 + PULSE_WIDTH_411us));

	//Enable Interrupt on FIFO almost full
	i2c_write(INTERRUPT_ENABLE1, A_FULL_EN);
	__delay_cycles(1000);

	//Return ID to allow to check if communication works
	return ID;
}

void max30102_read(int32_t *ledBuffer, uint8_t printFlag)
{
	//local count variable
	uint8_t a=0;

	//set samples to be read to 0 as default
	samples2read=0;

	//disable interrupts to avoid conflicts while reading new data
	__disable_interrupt();

	//Get read and write pointers
	wr_ptr = i2c_read(FIFO_WR_PTR);
	rd_ptr = i2c_read(FIFO_RD_PTR);

	//Calculates number of samples to be read
	if(rd_ptr < wr_ptr) samples2read = wr_ptr - rd_ptr;
	else if (rd_ptr > wr_ptr) samples2read = (32-rd_ptr) + wr_ptr;

	//If new samples available, start reading
	if(samples2read != 0)
	{
		//Read LED Buffers
		i2c_read_fifo();

		for(a=0;a<samples2read;a++)
		{
			//Send Data over UART if printFlag is set
			if(printFlag)
			{
				USCI_A_UART_transmitData(USCI_A1_BASE, led13Buffer[a]);
				USCI_A_UART_transmitData(USCI_A1_BASE, led12Buffer[a]);
				USCI_A_UART_transmitData(USCI_A1_BASE, led11Buffer[a]);
				USCI_A_UART_transmitData(USCI_A1_BASE, 0x00);
			}

			//Get 32Bit value out of 3 read Bytes
			ledBuffer[a] = (((int32_t)led11Buffer[a])<<16) + (((int32_t)led12Buffer[a])<<8) + ((int32_t)led13Buffer[a]);
		}
	}
	__enable_interrupt();
}

void max30102_readcontinous(int32_t *ledBuffer, uint8_t printFlag)
{
	//Discard first FIFO values
	while(max30102_int()==0){}
	max30102_read(ledBuffer, 0);

	while(1)
	{
		__delay_cycles(100);
		//Get new Data if Interrupt is set
		if(max30102_int()) max30102_read(ledBuffer, printFlag);
	}
}

void max30102_readDebug()
{
	__enable_interrupt();
	int32_t ledBuffer[BUFFERSIZE];
	i2c_write (MODE_CONFIGURATION_ADDR, MULTI_LED_MODE); //If not overwritten problem occurs after reentering from stop


	//Discard FIFO initialisiation Data
	while(max30102_int()==0){}
	max30102_read(ledBuffer, 0);

	while(startDebug_Flag==1 && debugDevice==MAX30102DEBUG) //Returning to Idle State if stop or other Device occurs
	{
		__delay_cycles(10);
		//__enable_interrupt();
		if(max30102_int()==1) max30102_read(ledBuffer, 1);
	}
}

uint8_t max30102_int()
{
	__disable_interrupt();

	if((GPIO_getInputPinValue(GPIO_PORT_P6, BIT2) == 0)) //Interrupt detected
	{
		__delay_cycles(10);
		i2c_read(0x00); //Reset Interrupt Register
		return 1;
	}
	__enable_interrupt();
	return 0;
}

uint8_t max30102_debugMode()
{
	//Disable ECG Chip
	ecg_en(0);
	//Enable LDO for MAX30102
	ldo1v8_en(1);
	__delay_cycles(400000);
	//initialize MAX30102
	max30102_init();
	return MAX30102DEBUG;
}
