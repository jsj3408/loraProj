/*
 * HAL_LORA.h
 *
 *  Created on: July 22, 2023
 *      Author: johns
 */



/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "app_config.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Macro section, defines with operation and function
*********************************************************************************************************************/

/**********************************************************************************************************************
* Typedef section
*********************************************************************************************************************/
typedef enum
{
	halLoraInvalidArgs = -2,
	halLoraFail,
	halLoraSuccess,
}halLoraRet_t;

typedef enum
{
	halLoraModeNone = 0,
	halLoraModeTX,
	halLoraModeRX,
	halLoraModeMax,
}halLoraMode_t;

typedef struct
{
	bool isInitialized;
	halLoraMode_t currentMode;
}halLoraCurrentInfo_t;

typedef struct
{
	uint8_t packetSNR;
	uint8_t packetRSSI;
}packetHealthInfo_t;
/**********************************************************************************************************************
* Enum section
*********************************************************************************************************************/


/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
halLoraRet_t halLoraInit(halLoraCurrentInfo_t * LoraCurrentInfo);
halLoraRet_t halLoraSetMode(halLoraMode_t mode, halLoraCurrentInfo_t * LoraCurrentInfo);
halLoraRet_t halLoraConfigTX(void);
halLoraRet_t halLoraConfigRX(void);
halLoraRet_t halLoraTransmit(uint8_t * payload, uint8_t length);
halLoraRet_t halLoraReadData(uint8_t * payload, uint8_t length);
halLoraRet_t halLoraReadNumBytes(uint8_t * numBytes);
halLoraRet_t halLoraBeginReceiveMode(halLoraCurrentInfo_t * LoraCurrentInfo);
halLoraRet_t halLoraGetPacketDiagnostics(packetHealthInfo_t * packetInfo);
void halLoraTXCompleteCB(void);
void halLoraRXPayloadCB(void);
