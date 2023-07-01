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

//#define USE_I2C
//#define GPIO_TOGGLE
#define USE_SPI
//#define LORA_TX
#define LORA_RX
//#define LED_TEST
/*
 * @brief   Application entry point.
 */

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

/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/
static void KL_InitPins(void);

/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/

int main(void)
{

    //status_t result = 1;
    uint8_t ret = 0;

    uint8_t loraAddr[] = {0x01, 0x0D, 0x10, 0x18};
    //each read operation will require two bytes of output
    uint8_t loraData[10] = {0};
    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }
	//SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    /* Init board hardware. */
	KL_InitPins(); //Initializing pins
    printf("i2c/SPI pins and GPIO pins initialized...I hope\n");

    /*BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();*/
//#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
//    BOARD_InitDebugConsole();
//#endif
/*
 * The below macro was used to test SPI on the BMP280.
 * The purpose is served, and hence I am removing it
 * */

#ifdef USE_SPI
    ret = spi_init();
    if(ret)
    {
    	lora_init();
#ifdef LORA_TX
    	lora_test_transmit();
#endif
#ifdef LORA_RX
    	lora_test_receive();
#endif
//    	for(int j=0; j < sizeof(loraAddr); j++)
//    	{
//    		ret = spi_transfer(SPI_Read, loraAddr[j], NULL, 1, &(loraData[2*j]));
//    		printf("ret value of SPI transfer:%d, Addr:0x%x data:0x%x\n",
//					ret, loraAddr[j], loraData[2*j]);
//    		if(0x01 == loraAddr[j])
//    		{
//    			printf("Mode is:%hu, LowFreqModeOn val:%hu\n",
//    					LongRangeMode(loraData[2*j]),
//    					LowFreqModeOn(loraData[2*j]));
//    		}
//    	}
//    	ret = spi_transfer(SPI_Read, *chip_addr, NULL, 2, chip_id_read);
//    	printf("ret value of SPI transfer:%d, data:%x\n", ret, chip_id_read[1]);
    }
#endif


#ifdef GPIO_TOGGLE
    while (retry--)
    {
    	SysTick_DelayTicks(1000U);
    	printf("Toggling...\n");
    	GPIO_TogglePinsOutput(GPIOE, 0x40000000); //toggling PTE30. bit30 is enabled
    }
#endif
    printf("Hello World\n");

    /* Force the counter to be placed into memory. */
//    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
//    while(1) {
//        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
//        __asm volatile ("nop");
//    }
    return 0 ;
}



/**
 * TODO: Change the location of this function later
 * */
static void KL_InitPins(void)
{

	//enabling clock for only PortE for I2C communication and GPIO use
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

	gpio_pin_config_t output_config =
	{
			kGPIO_DigitalOutput,
			0
	};
	//since the LED's are common cathode, we drive them Low to turn ON and High to turn OFF
	gpio_pin_config_t output_config_LED =
	{
			kGPIO_DigitalOutput,
			1
	};

	PORT_SetPinMux(PORTE, 30U, kPORT_MuxAsGpio);
	GPIO_PinInit(GPIOE, 30U, &output_config);
/*
	//Enable PortC for SPI communication
	CLOCK_EnableClock(kCLOCK_PortC);
	PORT_SetPinMux(PORTC, 4U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, 5U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, 6U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, 7U, kPORT_MuxAlt2);
	*/
	//use PORTD to enable SPI0 comms
	CLOCK_EnableClock(kCLOCK_PortD);
	//enable the SS/CS as GPIO only, the rest as i2c.
	PORT_SetPinMux(PORTD, 0U, kPORT_MuxAsGpio);
	GPIO_PinInit(GPIOD, 0U, &output_config);
	//PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt2);

	//enable clock for PortB to toggle on board LEDs
	CLOCK_EnableClock(kCLOCK_PortB);
	//Should be RED LED.
	PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio);
	//should be GREEN LED
	PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio);
	//set them as outputs
	GPIO_PinInit(GPIOB, 18U, &output_config_LED);
	GPIO_PinInit(GPIOB, 19U, &output_config_LED);

}
