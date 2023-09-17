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

#ifdef BAREMETAL
bool interrupt_issued = false;
#endif
/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
static halLoraCurrentInfo_t LORA_CurrentStatus = {0};
static packetHealthInfo_t LoraSignalData = {0};
static uint8_t readBuffer[LORA_READBUF_SIZE] = {0};
EventGroupHandle_t LORA_EventGroup;
TimerHandle_t payloadTXTimer;


extern volatile bool rxDataPresent;
extern volatile uint8_t * rxDataBuf;

/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/
static void payloadTX_CB(TimerHandle_t args);

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
#ifdef APP_RX
	//the device must be in RX mode by default
	if(halLoraSuccess != halLoraSetMode(halLoraModeRX, &LORA_CurrentStatus))
	{
		return App_LORA_fail;
	}
#endif
#ifdef APP_TX
	//set to TX mode for periodic transmission
	if(halLoraSuccess != halLoraSetMode(halLoraModeTX, &LORA_CurrentStatus))
	{
		return App_LORA_fail;
	}
#endif
	LORA_EventGroup = xEventGroupCreate();
	return ret;
}

void App_LORA_run(void * args)
{
	DB_PRINT(1, "Entered task: %s", __func__);
	EventBits_t bitSet = 0;
	uint8_t numBytesRX = 0;
	uint32_t payload_TX_iter = 1;
	char payload[] = "Eyyy: You've received this payload. Iteration:    .";
	char display_text[LORA_READBUF_SIZE];
#ifdef USE_DISPLAY
	uint8_t addon_text_len = 0;
	ssd1306_init();
	printf("Initialized SSD1306.....\n");
#endif
#ifdef APP_RX
	if(halLoraSuccess != halLoraBeginReceiveMode(&LORA_CurrentStatus))
	{
		DB_PRINT(1, "Unable to begin receive operation!");
	}
#endif
#ifdef APP_TX
	payloadTXTimer = xTimerCreate("TX_Timer", pdMS_TO_TICKS(15000), pdFALSE, NULL, payloadTX_CB);
	DB_PRINT(1, "Start timer of 15s");
	xTimerStart(payloadTXTimer, 100);
#endif
	for(;;)
	{
		DB_PRINT(1, "Task going to sleep");
		bitSet = xEventGroupWaitBits(LORA_EventGroup, TX_BIT | TX_DONE_BIT | RX_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
		//switch case because we cannot have both RX_BIT and TX_BIT
		DB_PRINT(1, "Received bitset value: %d", bitSet);
		switch(bitSet)
		{
			case RX_BIT:
				halLoraReadNumBytes(&numBytesRX);
				memset(readBuffer, 0, LORA_READBUF_SIZE);
				halLoraReadData(readBuffer, numBytesRX, &LoraSignalData);
#ifdef USE_DISPLAY
				memcpy(display_text, readBuffer, numBytesRX);
				//we want to display SNR and RSSI also in addition to payload received
				addon_text_len = snprintf(display_text+numBytesRX, 24, " SNR:%d, RSSI:%d", LoraSignalData.packetSNR, LoraSignalData.packetRSSI);
				ssd1306_write(display_text, numBytesRX+addon_text_len);
#endif
			break;

			case TX_BIT:
				sprintf(payload + ((sizeof(payload)-1) - 5), "%d", ++payload_TX_iter);
				if(halLoraSuccess != halLoraTransmit(&LORA_CurrentStatus,
										payload, (sizeof(payload)-1)))
				{
					DB_PRINT(1, "Transmit failed!");
				}
			break;

			case TX_DONE_BIT:
				DB_PRINT(1, "Transmission was successful. Restart timer!");
				xTimerStart(payloadTXTimer, 100);
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
#ifdef BAREMETAL
#ifdef LORA_TX
		lora_TX_complete_cb();
#endif

#ifdef LORA_RX
		lora_RX_response_cb();
#endif
		//temporarily set a variable that tells TX is complete
		interrupt_issued = true;
#else
		switch(LORA_CurrentStatus.currentMode)
		{
			case halLoraModeTX:
				halLoraTXCompleteCB();
				output = xEventGroupSetBitsFromISR(LORA_EventGroup, TX_DONE_BIT, &xHigherPriorityTaskWoken);
				if(pdFALSE != output)
				{
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				}
			break;

			case halLoraModeRX:
				halLoraRXPayloadCB();
				output = xEventGroupSetBitsFromISR(LORA_EventGroup, RX_BIT, &xHigherPriorityTaskWoken);
				if(pdFALSE != output)
				{
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				}
			break;

			default:
			break;
		}
#endif
		//set it back to 0
		//PORTD->PCR[5] = (PORTD->PCR[5] & ~PORT_PCR_ISF_MASK) | PORT_PCR_ISF(1);
		PORTD->ISFR = 0xFFFFFFFF;
	}
}

/**********************************************************************************************************************
* Local function definition section
*********************************************************************************************************************/
static void payloadTX_CB(TimerHandle_t args)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t output;
	output = xEventGroupSetBitsFromISR(LORA_EventGroup, TX_BIT, &xHigherPriorityTaskWoken);
	if(pdFALSE != output)
	{
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

}
