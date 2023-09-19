/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKL25Z4_Project.c
 * @brief   Application entry point.
 */
/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "MKL25Z4_Project.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
/*use macro UART0_BASE to use UART0 and UART2_BASE to use UART2*/
#define TESTUARTBASE	UART2_BASE


#if TESTUARTBASE == UART0_BASE
	#define TESTUART 		UART0
	#define UARTCLOCK kCLOCK_PllFllSelClk
	#define UARTIRQ UART0_IRQn
	#define UARTIRQ_HANDLER	UART0_IRQHandler
#else
	#define TESTUART 		UART2
	#define UARTCLOCK 		kCLOCK_BusClk
	#define UARTIRQ 		UART2_IRQn
	#define UARTIRQ_HANDLER	UART2_IRQHandler
#endif
/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Local enum section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
extern uint32_t SystemCoreClock;

volatile bool rxDataPresent = false;
volatile uint8_t rxDataBuf[10] = {0};
/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/
static void KL_InitPins(void);
static void ConfigureSystemClock(void);
static void UARTTest(void);
/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
/*******************************************************************************
 * @fn         main
 *
 * @brief      The function that is called at Startup
 *
 * @param[in]  void
 *
 * @return     int
 *
******************************************************************************/
int main(void)
{
    /* Init board hardware. */
	KL_InitPins(); //Initializing pins
	UARTTest();
    return 0;
}

/**********************************************************************************************************************
* Local function definition section
*********************************************************************************************************************/
/*******************************************************************************
 * @fn         KL_InitPins
 *
 * @brief      This function initializes all the GPIO pins and activates all the
 * 				Port Gate clocks
 *
 * @param[in]  void
 *
 * @return     void
 *
******************************************************************************/
static void KL_InitPins(void)
{
	gpio_pin_config_t output_config =
	{
			kGPIO_DigitalOutput,
			0
	};
	port_interrupt_t extern_interrupt =
	{
		kPORT_InterruptLogicOne,
	};
	//use PORTD to enable SPI0 comms
	CLOCK_EnableClock(kCLOCK_PortD);
	//enable the SS/CS as GPIO only, the rest as SPI.
	PORT_SetPinMux(PORTD, 0U, kPORT_MuxAsGpio);
	GPIO_PinInit(GPIOD, 0U, &output_config);
	//PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt2);

	//enable PTD5 as TX/RX interrupt
	PORT_SetPinMux(PORTD, 5U, kPORT_MuxAsGpio);
	PORT_SetPinInterruptConfig(PORTD, 5, extern_interrupt);
//	EnableIRQ(PORTD_IRQn);

#ifdef USE_DISPLAY
	//enabling clock for only PortE for I2C communication
	CLOCK_EnableClock(kCLOCK_PortE);
	port_pin_config_t config =
		{
			kPORT_PullUp,
		    kPORT_FastSlewRate,
		    kPORT_PassiveFilterDisable,
		    kPORT_OpenDrainEnable,
		    kPORT_LowDriveStrength,
			kPORT_MuxAlt6,
		    0,
		};
	PORT_SetPinConfig(PORTE, 0U, &config);
	PORT_SetPinConfig(PORTE, 1U, &config);	
#endif

	//since the LED's are common cathode, we drive them Low to turn ON and High to turn OFF
	gpio_pin_config_t output_config_LED =
	{
			kGPIO_DigitalOutput,
			1
	};
	//enable clock for PortB to toggle on board LEDs
	CLOCK_EnableClock(kCLOCK_PortB);
	//Should be RED LED.
	PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio);
	//should be GREEN LED
	PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio);
	//set them as outputs
	GPIO_PinInit(GPIOB, 18U, &output_config_LED);
	GPIO_PinInit(GPIOB, 19U, &output_config_LED);

	/*set up UART*/
#if TESTUARTBASE == UART0_BASE
	PORT_SetPinMux(PORTD, 7U, kPORT_MuxAlt4); //UART0TX
	PORT_SetPinMux(PORTD, 6U, kPORT_MuxAlt4); //UART0RX
#else
	//enabling clock for only PortE
	CLOCK_EnableClock(kCLOCK_PortE);
	PORT_SetPinMux(PORTE, 22U, kPORT_MuxAlt4); //UART2TX
	PORT_SetPinMux(PORTE, 23U, kPORT_MuxAlt4); //UART2RX
#endif
}


static void ConfigureSystemClock(void)
{
	//select FLL/PLL
	MCG->C1 |= MCG_C1_CLKS(0);
	//select the slow option
	MCG->C1 |= MCG_C1_IREFS(1);
	//here we want to select options that give us 48MHz output
	MCG->C4 |= MCG_C4_DRST_DRS(1);
	MCG->C4 |= MCG_C4_DMX32(1);
	//0b00: no divide. 0b01, divide by 2
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0b00);
	SystemCoreClockUpdate();
	DB_PRINT(1, "System core clock value is %lu Hz", SystemCoreClock);
}

static void UARTTest(void)
{
	ConfigureSystemClock();
    int ret = 0;
    uint32_t UARTClkFreq;
	uart_config_t uartConfig;
    UART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = 57600;
    uartConfig.enableTx = true;
    uartConfig.enableRx = true;
	//i've read that this value below is System clock div by 2.
	UARTClkFreq = CLOCK_GetFreq(UARTCLOCK);
	DB_PRINT(1, "UART Clock Freq is %lu Hz", UARTClkFreq);

#if TESTUARTBASE == UART0_BASE
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1); //we want UART freq to be div by 2.
#endif
//    SIM->SCGC4 |= SIM_SCGC4_UART2(1);//do i have to do this?
    ret = UART_Init(TESTUART, &uartConfig, UARTClkFreq);
    if(ret != kStatus_Success)
    {
    	DB_PRINT(1, "UART Init failed for baud %d with return code %d", uartConfig.baudRate_Bps, ret);
    	return;
    }
    /* Enable RX interrupt. */
    UART_EnableInterrupts(TESTUART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(UARTIRQ);
    uint32_t getUARTStatusFlag = UART_GetStatusFlags(TESTUART);
    while(1)
    {
		if(kUART_TxDataRegEmptyFlag & getUARTStatusFlag)
		{
			DB_PRINT(1, "Write something");
			UART_WriteByte(TESTUART, 'I');
			break;
		}
		else
		{
			DB_PRINT(1, "Status flag is %d", getUARTStatusFlag);
			while(1);
		}
    }
    DB_PRINT(1, "Enter the while loop to wait for any data");
    while(1)
    {
    	if(rxDataPresent == true)
    	{
    		DB_PRINT(1, "We have a new data! Data is 0x%x", rxDataBuf[0]);
    		rxDataPresent = false;
    	}
    }
}

void UARTIRQ_HANDLER(void)
{
    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(TESTUART))
    {
        rxDataBuf[0] = UART_ReadByte(TESTUART);
        rxDataPresent = true;
    }
}

