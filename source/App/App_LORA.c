/*
 * App_LORA.c
 *
 *  Created on: Aug 2, 2023
 *      Author: johns
 */

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "App_LORA.h"
/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
#define LORA_READBUF_SIZE	256
/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
static halLoraCurrentInfo_t LORA_CurrentStatus = {0};
static uint8_t readBuffer[LORA_READBUF_SIZE] = {0};
EventGroupHandle_t LORA_EventGroup;
/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
App_LORA_ret_t App_LORA_init(void)
{
	App_LORA_ret_t ret = App_LORA_success;
	//TODO: Add checks and fail cases inside this function
	if(halLoraSuccess != halLoraInit(&LORA_CurrentStatus))
	{
		return App_LORA_fail;
	}
	//the device must be in RX mode by default
//	if(halLoraSuccess != halLoraSetMode(halLoraModeRX, &LORA_CurrentStatus))
//	{
//		return App_LORA_fail;
//	}
	halLoraConfigRX();
	LORA_EventGroup = xEventGroupCreate();
	return ret;
}

void App_LORA_run(void * args)
{
	DB_PRINT(1, "Entered task: %s", __func__);
	EventBits_t bitSet = 0;
	uint8_t numBytesRX = 0;
	halLoraBeginReceiveMode(&LORA_CurrentStatus);
	for(;;)
	{
		DB_PRINT(1, "Task going to sleep");
		vTaskDelay(1000);
		bitSet = xEventGroupWaitBits(LORA_EventGroup, TX_BIT | RX_BIT, pdTRUE, pdFALSE, 3000);
		//switch case because we cannot have both RX_BIT and TX_BIT
		DB_PRINT(1, "Received bitset value: %d", bitSet);
		switch(bitSet)
		{
			case RX_BIT:
				halLoraReadNumBytes(&numBytesRX);
				memset(readBuffer, 0, LORA_READBUF_SIZE);
				halLoraReadData(readBuffer, numBytesRX);
			break;

			default:
				DB_PRINT(1, "Un-handled case here, bruh.");
			break;
		}
	}
}

/*******************************************************************************
 * @fn         PORTD_IRQHandler
 *
 * @brief      This is the CB function triggered when an Interrupt occurs on PTD.
 * 				It first switches off the Interrupt by writinh into ISFR and then
 * 				calls the necessary function to respond to the interrupt
 *
 * @param[in]  void
 *
 * @return     void
 *
******************************************************************************/
void PORTD_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t output;
	if((PORTD->PCR[5] & PORT_PCR_ISF_MASK) != 0)
	{
		//set it back to 0
		//PORTD->PCR[5] = (PORTD->PCR[5] & ~PORT_PCR_ISF_MASK) | PORT_PCR_ISF(1);
		PORTD->ISFR = 0xFFFFFFFF;
		halLoraRXPayloadCB();
		//temporarily set a variable that tells TX is complete
		output = xEventGroupSetBitsFromISR(LORA_EventGroup, RX_BIT, &xHigherPriorityTaskWoken);
		if(pdFALSE != output)
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
	}
}

