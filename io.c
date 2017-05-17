/*
 * io.c
 *
 *  Created on: 24.11.2016
 *      Author: student
 */

#include "max30102.h"


void clock_init()
{

	//Initialize external Quarz
	UCS_turnOnLFXT1WithTimeout(UCS_XT1_DRIVE_3, UCS_XCAP_0, 50000);
	UCS_setExternalClockSource(32768, 0);
	UCS_initClockSignal(UCS_FLLREF, UCS_XT1CLK_SELECT, UCS_CLOCK_DIVIDER_1);

	//Set SMCLK to 4MHz (DCO/2)
	UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLKDIV_SELECT, UCS_CLOCK_DIVIDER_2);

	//Set ACLK to 32,778kHz (external Quarz)
	UCS_initClockSignal(UCS_ACLK, UCS_XT1CLK_SELECT, UCS_CLOCK_DIVIDER_1);

	//Set MCLK to 8 MHz (DCO)
	UCS_initClockSignal(UCS_MCLK, UCS_DCOCLKDIV_SELECT, UCS_CLOCK_DIVIDER_1);

	//Initialize DCO to 8MHz (8Mhz/32,768kHz = 244)
	UCS_initFLLSettle(8000,244);
}


void gpio_init()
{
	//Initialize global Flags and Variables
	startDebug_Flag = 0;
	enableDebug_Flag = 0;
	debugDevice = 0;
	liveMode_Flag = 0;
	debug_led(0);

	//init NFC Data
	nfc_samplecounter=0;
	uint8_t a=0;
	for(a=0;a<12;a++){nfc_data[a] = 0;}

	//Not actually used GPIO Pins Port1
	//Coming: PPG_INT -> P1.0, NFC_IRQ -> P1.1
	GPIO_setAsOutputPin(
	   	GPIO_PORT_P1,
	   	GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
	    GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7
	);
	GPIO_setOutputLowOnPin(
	    GPIO_PORT_P1,
	    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 +
	    GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7
	);

	//Not actually used GPIO Pins on Port3
	GPIO_setAsOutputPin(
	   	GPIO_PORT_P3,
		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
	);
	GPIO_setOutputLowOnPin(
		GPIO_PORT_P3,
		GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2
	);


	//Port4: Power_selection an unused Pins 0 and 3
	GPIO_setAsInputPin(
		GPIO_PORT_P4,
		Power_sel
	);
	GPIO_setAsOutputPin(
		GPIO_PORT_P4,
		GPIO_PIN0 + GPIO_PIN3
	);
	GPIO_setOutputLowOnPin(
		GPIO_PORT_P4,
		GPIO_PIN0 + GPIO_PIN3
	);

	//GPIO Pins Port5
	GPIO_setAsOutputPin(
	   	GPIO_PORT_P5,
	   	ECG_Npwdn + ECG_start + DebugLED + PPG_en
	);
	GPIO_setOutputLowOnPin(
	    GPIO_PORT_P5,
		ECG_Npwdn + ECG_start + DebugLED + PPG_en
	);

	//GPIO Pins Port6
	//Notiz: PPG_int, NFC_irq noch an GPIO pins, müssen Hardwaremäßig an P1.0 und P1.1 (wie in NFC_Pulsmesser_final) gelegt werden, um Port Interrupts zu nutzen
	GPIO_setAsInputPin(
	   	GPIO_PORT_P6,
	   	NFC_irq + PPG_int + Electr_det + ECG_Ndrdy
	);
	GPIO_setAsOutputPin(
	   	GPIO_PORT_P6,
	   	NFC_ss + ECG_Ncs
	);
	GPIO_setOutputHighOnPin(
	    GPIO_PORT_P6,
		NFC_ss + ECG_Ncs
	);
}

void timer_init()
{
	//Start timer in continuous mode sourced by SMCLK/64 = 62500Hz
	Timer_A_initContinuousModeParam initContParam = {0};
	initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
	initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
	initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	initContParam.timerClear = TIMER_A_DO_CLEAR;
	initContParam.startTimer = false;
	Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

	//Initiaze compare mode
	Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

	Timer_A_initCompareModeParam initCompParam = {0};
	initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
	initCompParam.compareInterruptEnable =
	TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
	initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;

	//Set Compare Register to 62500 -> 1Hz
	initCompParam.compareValue = 62500;

	//Initialize and Start Timer
	Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);
	Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
}


void debug_led(uint8_t b)
{
	//Switch Green LED on Debugging Board
	if(b==1)//On
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P5, DebugLED);
		debug_led_state = 1;
	}
	else if(b==0)//Off
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P5, DebugLED);
		debug_led_state = 0;
	}
}


void ldo1v8_en(uint8_t b)
{
	//Switch 1,8V LDO for MAX30102
	if(b==1) GPIO_setOutputHighOnPin(GPIO_PORT_P5, PPG_en);	//On
	else GPIO_setOutputLowOnPin(GPIO_PORT_P5, PPG_en);		//Off
}


void ecg_en(uint8_t b)
{
	//Switch ECG Chip
	if(b==1) GPIO_setOutputHighOnPin(GPIO_PORT_P5, ECG_Npwdn);	//On
	else GPIO_setOutputLowOnPin(GPIO_PORT_P5, ECG_Npwdn);		//Off
}


void ecg_start(uint8_t b)
{
	//Set ECG Start Pin
	if(b==1) GPIO_setOutputHighOnPin(GPIO_PORT_P5, ECG_start);	//High
	else GPIO_setOutputLowOnPin(GPIO_PORT_P5, ECG_start);		//Low
}


void uart_init()
{
	//256000 Baud
	//Rechner: software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

	//Configure Port Pins
	GPIO_setAsPeripheralModuleFunctionInputPin(
		GPIO_PORT_P4,
	    GPIO_PIN4 + GPIO_PIN5
	);

	//Configure UART Interface
	USCI_A_UART_initParam param = {0};
	param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar = 15;
	param.firstModReg = 0;
	param.secondModReg = 5;
	param.parity = USCI_A_UART_NO_PARITY;
	param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
	param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	param.uartMode = USCI_A_UART_MODE;
    param.overSampling = 0;

    if(STATUS_FAIL == USCI_A_UART_init(USCI_A1_BASE, &param))
    {
        return;
    }

    //Enable UART module for operation
    USCI_A_UART_enable(USCI_A1_BASE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt(USCI_A1_BASE,
                               USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A1_BASE,
                                USCI_A_UART_RECEIVE_INTERRUPT);
}


void uart_sendc(uint8_t c)
{
	//Send character over UART
	USCI_A_UART_transmitData(USCI_A1_BASE, c);
	//Line Terminator
	USCI_A_UART_transmitData(USCI_A1_BASE, '\r');
	USCI_A_UART_transmitData(USCI_A1_BASE, '\n');
}


void uart_sends(char* s)
{
	//Send String over UART
	while (*s)
	{
		USCI_A_UART_transmitData(USCI_A1_BASE, *s);
	    s++;
	}
	//Line Terminator
	USCI_A_UART_transmitData(USCI_A1_BASE, '\r');
	USCI_A_UART_transmitData(USCI_A1_BASE, '\n');
}


void spi_init(uint32_t spiclk)
{
    //Initilize SPI Pins
	GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN7
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
	    GPIO_PORT_P3,
	    GPIO_PIN3 + GPIO_PIN4
    );

    //Initialize Master
    USCI_A_SPI_initMasterParam param = {0};
    param.selectClockSource = USCI_A_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = UCS_getSMCLK();
    param.desiredSpiClock = spiclk;
    param.msbFirst = USCI_A_SPI_MSB_FIRST;
    param.clockPhase = USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    uint8_t returnValue = USCI_A_SPI_initMaster(USCI_A0_BASE, &param);

    if(STATUS_FAIL == returnValue)
    {
        return;
    }

    //Enable SPI module
    USCI_A_SPI_enable(USCI_A0_BASE);
}


void spi_write(uint8_t byte)
{
	//CS Low
	P6OUT &= ~BIT5;
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
    //Transmit Data to slave
	USCI_A_SPI_transmitData(USCI_A0_BASE, byte);
	//Wait While SPI is Busy
	while(USCI_A_SPI_isBusy(USCI_A0_BASE));
	//CS High
    P6OUT |= BIT5;
}


uint8_t spi_read(uint8_t reg)
{
	uint8_t receivedData=10;
	//CS low
	P6OUT &= ~BIT5;
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, (0x20 | reg));
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	receivedData = USCI_A_SPI_receiveData(USCI_A0_BASE);
	//CS High
	P6OUT |= BIT5;
	return receivedData;
}


void spi_send2b(uint8_t b1, uint8_t b2)
{
	P6OUT &= ~BIT5;                  // CS enabled
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, b1);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, b2);
	while(USCI_A_SPI_isBusy(USCI_A0_BASE));
    P6OUT |= BIT5;                   // CS disabled
}


void spi_send3b(uint8_t b1, uint8_t b2, uint8_t b3)
{
	P6OUT &= ~BIT5;                  // CS enabled
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, b1);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	USCI_A_SPI_transmitData(USCI_A0_BASE, b2);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
		USCI_A_SPI_transmitData(USCI_A0_BASE, b3);
	while(USCI_A_SPI_isBusy(USCI_A0_BASE));
    P6OUT |= BIT5;                   // CS disabled
}


void spi_read_continous(uint8_t *dataBuffer1, uint8_t *dataBuffer2, uint8_t *statusBuffer)
{
	P6OUT &= ~BIT5;                  // CS enabled

	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
	while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
	statusBuffer[2] = USCI_A_SPI_receiveData(USCI_A0_BASE);
	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
		while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
		statusBuffer[1] = USCI_A_SPI_receiveData(USCI_A0_BASE);
		USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
			while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
			statusBuffer[0] = USCI_A_SPI_receiveData(USCI_A0_BASE);

	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
		while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
		dataBuffer1[2] = USCI_A_SPI_receiveData(USCI_A0_BASE);
		USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
			while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
			dataBuffer1[1] = USCI_A_SPI_receiveData(USCI_A0_BASE);
			USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
				while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
				dataBuffer1[0] = USCI_A_SPI_receiveData(USCI_A0_BASE);

	USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
			while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
			dataBuffer2[2] = USCI_A_SPI_receiveData(USCI_A0_BASE);
			USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
				while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
					dataBuffer2[1] = USCI_A_SPI_receiveData(USCI_A0_BASE);
					USCI_A_SPI_transmitData(USCI_A0_BASE, 0x00);
					while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));
					dataBuffer2[0] = USCI_A_SPI_receiveData(USCI_A0_BASE);
					while(!USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT));

	P6OUT |= BIT5;                   // CS disabled
}


void i2c_init(uint8_t slave_address)
{
	__delay_cycles(400000);
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN2); 	//SCL
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN1); 	//SDA

 	GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
 	GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);

	// Initialize Master
	USCI_B_I2C_initMasterParam param = {0};
	param.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
	param.i2cClk = UCS_getSMCLK();
	param.dataRate = USCI_B_I2C_SET_DATA_RATE_400KBPS;
	USCI_B_I2C_initMaster(USCI_B1_BASE, &param);

	// Specify slave address
	USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, slave_address);
	// Set in transmit mode
	USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_MODE);
	//Enable USCI B I2C Module to start operations
	USCI_B_I2C_disableInterrupt(USCI_B1_BASE, 0xFF);
	USCI_B_I2C_enable(USCI_B1_BASE);

	__delay_cycles(1000);
}


void i2c_write(uint8_t reg, uint8_t data)
{
	USCI_B_I2C_masterSendMultiByteStartWithTimeout (USCI_B1_BASE, reg,10000);
	USCI_B_I2C_masterSendMultiByteFinishWithTimeout (USCI_B1_BASE, data,10000);
	while(USCI_B_I2C_isBusBusy (USCI_B1_BASE));
}


uint8_t i2c_read(uint8_t reg)
{
	uint8_t I2Creceive=0;
	uint16_t timeout = 1000;

	USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B1_BASE, reg,1000);
	while((!(HWREG8(USCI_B1_BASE + OFS_UCBxIFG) & UCTXIFG)) && --timeout)
    {
         ;
    }

	if(timeout == 0)
    {
        return (STATUS_FAIL);
    }

	USCI_B_I2C_masterReceiveSingleStartWithTimeout(USCI_B1_BASE,1000);// Hier wird das Byte empfangen was gesendet wurde. bezogen auf die Base adresse.
	I2Creceive =  USCI_B_I2C_masterReceiveSingle(USCI_B1_BASE);// Hier wird das Byte empfangen was gesendet wurde. bezogen auf die Base adresse.
	while(USCI_B_I2C_isBusBusy (USCI_B1_BASE));
	return I2Creceive;
}

uint8_t i2c_read_fifo()
{
	uint16_t timeout = 1000;
	uint8_t a=0;

 	//USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B1_BASE,0x07,1000);
	USCI_B_I2C_masterSendMultiByteStartWithTimeout(USCI_B1_BASE, 0x07 ,1000);
 	while((!(HWREG8(USCI_B1_BASE + OFS_UCBxIFG) & UCTXIFG)) && --timeout)
	{
	      ;
	}

	if(timeout == 0)
	{
	   return (STATUS_FAIL);
	}

	USCI_B_I2C_masterReceiveMultiByteStart(USCI_B1_BASE);

	//Poll for Start bit to complete
	while(HWREG8(USCI_B1_BASE + OFS_UCBxCTL1) & UCTXSTT)
	{
	    ;
	}

	for(a=0;a<samples2read;a++)
	{
		led11Buffer[a] = USCI_B_I2C_masterReceiveSingle(USCI_B1_BASE);
		led12Buffer[a] = USCI_B_I2C_masterReceiveSingle(USCI_B1_BASE);

	    if(a == (samples2read-1))
	    		{
	    	    	USCI_B_I2C_masterReceiveMultiByteStop(USCI_B1_BASE);
	    	    	//Wait for Stop to finish
	    	    	while(HWREG8(USCI_B1_BASE + OFS_UCBxCTL1) & UCTXSTP)
	    	    	{
	    	    		;
	    	    	}
	    		}

	    led13Buffer[a] = USCI_B_I2C_masterReceiveSingle(USCI_B1_BASE);

	}

	return (STATUS_SUCCESS);
}

void debug_idle()
{
	//IDLE State for Debugging Mode (While waiting for initialisation data or start signal)
	while(enableDebug_Flag)
	{
		__delay_cycles(10);
		if(startDebug_Flag)
		{
			if(debugDevice == MAX30102DEBUG) max30102_readDebug();
			else if(debugDevice == ADS1292DEBUG) ads1292_readDebug();
			else if(debugDevice == 0x00) startDebug_Flag = 0;
		}
	}
}

//******************************************************************************
//
//This is the USCI_A1 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void USCI_A1_ISR(void)
{
	__disable_interrupt();

	uint8_t r;

    switch(__even_in_range(UCA1IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
    	r = USCI_A_UART_receiveData(USCI_A1_BASE);
    	switch(r)
    	{
    	////////////////////////////////////////////////////////
    	/*Befehlsliste für die Steuerung per MATLAB GUI:

    	1     MAX30102 einschalten und initialisieren
    	31    ADS1292 einschalten und initialisieren
    	100   Daten von uC senden starten
    	101   Daten von uC senden stoppen
    	102   Porttest(GUI)/Debugmode(uC) starten
    	104	  activate Life Mode
    	105	  deactivate Life Mode
    	23 / 47   Debugmode(uC) stoppen

    	2     'MAX30102 SR:  50 SPS'
    	3     'MAX30102 SR: 100 SPS'
    	4     'MAX30102 SR: 200 SPS'
    	5     'MAX30102 SR: 400 SPS'
    	6     'MAX30102 SR: 800 SPS'
    	7     'MAX30102 SR: 1.0 kSPS'
    	8     'MAX30102 SR: 1.6 kSPS'
    	9     'MAX30102 SR: 3.2 kSPS'
    	10    'MAX30102 LED-Strom:  0.2 mA'
    	11    'MAX30102 LED-Strom:  0.4 mA'
    	12    'MAX30102 LED-Strom:  3.1 mA'
    	13    'MAX30102 LED-Strom:  6.4 mA'
    	14    'MAX30102 LED-Strom: 12.5 mA'
    	15    'MAX30102 LED-Strom: 25.4 mA'
    	16    'MAX30102 LED-Strom: 50.0 mA'
    	17    'MAX30102 Mittelung:  kein Averaging
    	18    'MAX30102 Mittelung:  Averaging 2 Samples'
    	19    'MAX30102 Mittelung:  Averaging 4 Samples'
    	20    'MAX30102 Mittelung:  Averaging 8 Samples'
    	21    'MAX30102 Mittelung:  Averaging 16 Samples'
    	22    'MAX30102 Mittelung:  Averaging 32 Samples'

    	32    'ADS1292 SR:  125 SPS'
    	33    'ADS1292 SR: 250 SPS'
    	34    'ADS1292 SR: 500 SPS'
    	35    'ADS1292 SR: 1.0 kSPS'
    	36    'ADS1292 SR: 2.0 kSPS'
    	37    'ADS1292 SR: 4.0 kSPS'
    	38    'ADS1292 SR: 8.0 kSPS'
    	39    'ADS1292 Verstärkungsfaktor:  1'
    	40    'ADS1292 Verstärkungsfaktor:  2'
    	41    'ADS1292 Verstärkungsfaktor:  3'
    	42    'ADS1292 Verstärkungsfaktor:  4'
    	43    'ADS1292 Verstärkungsfaktor:  6'
    	44    'ADS1292 Verstärkungsfaktor:  8'
    	45    'ADS1292 Verstärkungsfaktor: 12'
    	46    'ADS1292 Testsignal: Rechteck'*/
    	////////////////////////////////////////////////////////


    	/////**************************************************
    	////MAX30102 Configuration
    	case 1: //MAX30102 einschalten und initialisieren
    		debugDevice = max30102_debugMode();
    		break;

    	case 2: //MAX30102 SR:  50 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_50);
    		break;

    	case 3: //MAX30102 SR:  100 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_100);
    	    break;

    	case 4: //MAX30102 SR:  200 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_200);
    	    break;

    	case 5: //MAX30102 SR:  400 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_400);
    	    break;

    	case 6: //MAX30102 SR:  800 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_800);
    	    break;

    	case 7: //MAX30102 SR:  1k SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_1k);
    	    break;

    	case 8: //MAX30102 SR:  1k6 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_1k6);
    	    break;

    	case 9: //MAX30102 SR:  3k2 SPS
    		i2c_write (SPO2_CONFIGURATION, MAX_SMP_RATE_3k2);
    	    break;

    	case 10: //MAX30102 LED-Strom:  0.2 mA
    		i2c_write (LED_RED, LED_0mA2);
    	    break;

    	case 11: //MAX30102 LED-Strom:  0.4 mA
    	    i2c_write (LED_RED, LED_0mA4);
    	    break;

    	case 12: //MAX30102 LED-Strom:  3.1 mA
    	    i2c_write (LED_RED, LED_3mA1);
    	    break;

    	case 13: //MAX30102 LED-Strom:  6.4 mA
    	    i2c_write (LED_RED, LED_6mA4);
    	    break;

    	case 14: //MAX30102 LED-Strom:  12.5 mA
    	    i2c_write (LED_RED, LED_12mA5);
    	    break;

    	case 15: //MAX30102 LED-Strom:  25.4 mA
    	    i2c_write (LED_RED, LED_25mA4);
    	    break;

    	case 16: //MAX30102 LED-Strom:  50 mA
    	    i2c_write (LED_RED, LED_50mA);
    	    break;

    	case 17: //MAX30102 Mittelung:  kein Averaging
    		i2c_write (FIFO_CONFIGURATION, (SMP_AVG_1 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;

    	case 18: //MAX30102 Mittelung:  Averaging 2 Samples
    	    i2c_write (FIFO_CONFIGURATION, (SMP_AVG_2 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;

    	case 19: //MAX30102 Mittelung:  Averaging 4 Samples
    	    i2c_write (FIFO_CONFIGURATION, (SMP_AVG_4 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;

    	case 20: //MAX30102 Mittelung:  Averaging 8 Samples
    	    i2c_write (FIFO_CONFIGURATION, (SMP_AVG_8 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;

    	case 21: //MAX30102 Mittelung:  Averaging 16 Samples
    	    i2c_write (FIFO_CONFIGURATION, (SMP_AVG_16 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;

    	case 22: //MAX30102 Mittelung:  Averaging 32 Samples
    	    i2c_write (FIFO_CONFIGURATION, (SMP_AVG_32 + FIFO_ROLLOVER_EN + (32-BUFFERSIZE)));
    	    break;
    	/////***********************************************************************************

    	case 23: //Exit Debugging Mode
    	    enableDebug_Flag = 0;
    	    debugDevice = max30102_debugMode(); //Init state for normal operation
    	    debugDevice = 0;
    	    startDebug_Flag = 0;
    	    break;

    	/////***********************************************************************************
    	/////ADS1292 Configuration
    	case 31: //ADS1292 einschalten und initialisieren
    		debugDevice = ads1292_debugMode();
    		break;

    	case 32: //ADS1292 SR:  125 SPS
    		spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_125);
    	    break;


    	case 33: //ADS1292 SR:  250 SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_250);
    	    break;

    	case 34: //ADS1292 SR:  500 SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_500);
    	    break;

    	case 35: //ADS1292 SR:  1k SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_1k);
    	    break;

    	case 36: //ADS1292 SR:  2k SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_2k);
    	    break;

    	case 37: //ADS1292 SR:  4k SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_4k);
    	    break;

    	case 38: //ADS1292 SR:  8k SPS
    	    spi_send3b(0b01000010, 0x00, ADS_SMP_RATE_8k);
    	    break;

    	case 46: //ADS1292 Testsignal: Rechteck
    		ads1292_test();
    		break;

    	/////***********************************************************************************

    	case 47: //Exit Debugging Mode
    	    enableDebug_Flag = 0;
    	    debugDevice = max30102_debugMode(); //Init state for normal operation
    	    debugDevice = 0;
    	    startDebug_Flag = 0;
    	    break;


    	/////***********************************************************************************
    	/////General commands
    	case 100: //Start Sending Data
    		if(debugDevice != 0x00) startDebug_Flag = 1;	//Start Button pressed before initialisation
    		break;

    	case 101: //Stop Sending Data
    		startDebug_Flag = 0;
    		break;

    	case 102: //Enter Debugging Mode
    		enableDebug_Flag = 1;
    		debugDevice = max30102_debugMode(); //Default Device: MAX30102
    	    USCI_A_UART_transmitData(USCI_A1_BASE, 102);  //Acknowledge to MATLAB GUI
    	   	break;

    	case 103: //Exit Debugging Mode
    		enableDebug_Flag = 0;
    		debugDevice = max30102_debugMode(); //Init state for normal operation
    		debugDevice = 0;
    		startDebug_Flag = 0;
    		break;
    	case 104:
    		liveMode_Flag = 1;
    		enableDebug_Flag = 0;
    		break;
    	case 105:
    		liveMode_Flag = 0;
    	/////***********************************************************************************
    	}
       __delay_cycles(1000000);
        break;
    default: break;
    }
}

//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_A0_ISR(void)
{
	as3955_nfc_transmit(nfc_data);
	as3955_IRQ();

	nfc_samplecounter=0;

	uint8_t a=0;
	for(a=0;a<12;a++){nfc_data[a]=0;}

}
