/*
 * HAL_LORA.h
 *
 *  Created on: July 22, 2023
 *      Author: johns
 */



/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "HAL_SPI.h"
#include "app_config.h"

#ifdef USE_DISPLAY
#include "i2c_comm.h"
#include "ssd1306.h"
#include "bitmap_array.h"
#endif

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
	int16_t packetSNR;
	int16_t packetRSSI;
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
halLoraRet_t halLoraTransmit(halLoraCurrentInfo_t * LoraCurrentInfo,
							uint8_t * payload_data, uint8_t payload_len);
halLoraRet_t halLoraReadData(uint8_t * RX_buffer, uint8_t payload_len, packetHealthInfo_t * packetHealthInfo);
halLoraRet_t halLoraReadNumBytes(uint8_t * numBytes);
halLoraRet_t halLoraBeginReceiveMode(halLoraCurrentInfo_t * LoraCurrentInfo);
halLoraRet_t halLoraGetPacketDiagnostics(packetHealthInfo_t * packetInfo);
void halLoraTXCompleteCB(void);
void halLoraRXPayloadCB(void);
