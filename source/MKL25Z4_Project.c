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
#include "App_LORA.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Local enum section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
uint8_t status;
status_t result = kStatus_Success;
extern bool interrupt_issued;

TaskHandle_t * handle = NULL;

extern uint32_t SystemCoreClock;
/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/
static void KL_InitPins(void);
static void LoraPOCTestFunction(void);
static void ConfigureSystemClock(void);
static void sampleTask(void * args);
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
#ifdef BAREMETAL
	LoraPOCTestFunction();
#else
	//setting to 32KHz since we want low power consumption
	ConfigureSystemClock();
	KL_InitPins();
	if(false == spi_init())
	{
		DB_PRINT(1, "Couldn't initialize SPI. exit!");
		return 0;
	}
	if(App_LORA_success != App_LORA_init())
	{
		DB_PRINT(1, "Lora Init failed!");
		return 0;
	}
	if(pdFALSE == xTaskCreate(App_LORA_run, "App_LORA_task", configMINIMAL_STACK_SIZE+0x300, NULL, 1,
			handle))
	{
		DB_PRINT(1, "Starting LORA task failed!");
	}
	else
	{
		vTaskStartScheduler();
	}
#endif
    return 0;
}


//static void sampleTask(void * args)
//{
//	for(;;)
//	{
//		DB_PRINT(1, "You are officially running this task!");
//		vTaskDelay(pdMS_TO_TICKS(1000));
//	}
//}

/**********************************************************************************************************************
* Local function definition section
*********************************************************************************************************************/
/*******************************************************************************
 * @fn         LoraPOCTestFunction
 *
 * @brief      This function tests the functionality of two LORA modules powered
 * 				by two FRDM-KL25 MCUs, one acting as the transmitter and another
 * 				as a receiver.
 * 				Two FW builds need to be done for TX and RX.
 * 				Enable macro LORA_TX and disable macro LORA_RX for FW to be
 * 				flashed into one MCU acting as a TX. Enable macro LORA_RX and
 * 				disable LORA_TX and create the build to be flashed into another
 * 				MCU acting as receiver.
 * 				When TX is done, the white LED will turn to blue and when RX is
 * 				received the white LED will turn to blue or otherwise remain white.
 *
 * @param[in]  void
 *
 * @return     void
 *
******************************************************************************/
static void LoraPOCTestFunction(void)
{
    //status_t result = 1;
    uint8_t ret = 0;
    uint8_t loraAddr[] = {0x01, 0x0D, 0x10, 0x18};
    //each read operation will require two bytes of output
    uint8_t loraData[10] = {0};

    /* Init board hardware. */
	KL_InitPins(); //Initializing pins
    printf("i2c/SPI pins and GPIO pins initialized...I hope\n");
    ret = spi_init();
    if(ret)
    {
    	lora_init();
#ifdef LORA_TX
    	lora_test_transmit();
//    	while(interrupt_issued == false)
//    	{
//    		DB_PRINT(1, "waiting for TX interrupt");
//    		SysTick_DelayTicks(500U);
//    	}
#endif
#ifdef LORA_RX
    	lora_test_receive();
//    	while(interrupt_issued == false)
//    	{
//    		DB_PRINT(1, "waiting for RX interrupt");
//    		SysTick_DelayTicks(500U);
//    	}
#endif
    }
    printf("***************End of program execution. Now wait for interrupts**************");
}

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
	EnableIRQ(PORTD_IRQn);

#ifdef BAREMETAL
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
	//divide by 16 to get 3MHz.
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(0b1111);
	SystemCoreClockUpdate();
	DB_PRINT(1, "System core clock value is %lu Hz", SystemCoreClock);
}
