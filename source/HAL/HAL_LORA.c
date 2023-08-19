/*
 * HAL_LORA.c
 *
 *  Created on: July 22, 2023
 *      Author: johns
 */

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include <sx1278.h>
#include "HAL_LORA.h"
#include "spi_comm.h"
/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
#define READ_REGS	0
#define WRITE_REGS	0
#define CRYSTAL_OSC_FREQ 	32	//in MHz
#define CARRIER_FREQ		434 //in MHz

#define SPREAD_FACTOR 12 //can be from 7-12. If 6, then we have to add extra configurations

/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
extern halSPIAction_t * SPI_handle;

/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/
static void calculateSignalHealth(uint8_t snrVal, uint8_t rssiVal, packetHealthInfo_t * packetHealthInfo);

/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
/*******************************************************************************
 * @fn         halLoraInit
 *
 * @brief      This function initializes LORA with communicating Freq, operation mode, etc
 *
 * @param[in]  halLoraCurrentInfo_t * LoraCurrentInfo: structure to save current LORA
 * 				status
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraInit(halLoraCurrentInfo_t * LoraCurrentInfo)
{
	uint8_t data[2] = {0};
	// put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value to write in REG_OPMODE is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);

	RegFrData regFreq = {0};
	//write the required carrier frequency into the LORA register
	lora_calculateRegFrequency(CARRIER_FREQ, CRYSTAL_OSC_FREQ, &regFreq);
	(void) SPI_handle->halSPIWrite(REG_FR_MSB, 1, &(regFreq.FrMSB));
	(void) SPI_handle->halSPIWrite(REG_FR_MID, 1, &(regFreq.FrMID));
	(void) SPI_handle->halSPIWrite(REG_FR_LSB, 1, &(regFreq.FrLSB));
	//TODO: Add validation checks.
	LoraCurrentInfo->isInitialized = true;
	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraSetMode
 *
 * @brief      This function sets up lora to be TX or RX
 *
 * @param[in]  halLoraCurrentInfo_t * LoraCurrentInfo: structure to save current LORA
 * 				status
 *
 * @param[in]  halLoraMode_t mode: mode to switch into
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraSetMode(halLoraMode_t mode, halLoraCurrentInfo_t * LoraCurrentInfo)
{
	halLoraRet_t ret = halLoraSuccess;
	switch(mode)
	{
		case halLoraModeTX:
			if(halLoraConfigTX() != halLoraSuccess)
			{
				ret = halLoraFail;
			}
			else
			{
				LoraCurrentInfo->currentMode = halLoraModeTX;
			}
		break;

		case halLoraModeRX:
			if(halLoraConfigRX() != halLoraSuccess)
			{
				ret = halLoraFail;
			}
			else
			{
				LoraCurrentInfo->currentMode = halLoraModeRX;
			}
		break;

		default:
			ret = halLoraInvalidArgs;
		break;
	}
	return ret;
}

/*******************************************************************************
 * @fn         halLoraConfigTX
 *
 * @brief      This function sets up all the required params to configure LORA as TX
 *
 * @param[in]  void
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraConfigTX(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	//make sure OpMode is in sleep mode = 0
	(void) SPI_handle->halSPIRead(REG_OPMODE, 1, data);
	DB_PRINT(1, "Value in REG_OPMODE is:%hu", data[0]);
	DB_PRINT(1, "LongRangeMode:%hu, AccessSharedReg:%hu, LowfreqModeOn:%hu"
			"CurrentMode:%hu",
			GET_VAL(data[0], LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN),
			GET_VAL(data[0], ACCSHAREDREG_SHIFT, ACCSHAREDREG_BITLEN),
			GET_VAL(data[0], LOWFREQMODEON_SHIFT, LOWFREQMODEON_BITLEN),
			GET_VAL(data[0], DEVICEMODE_SHIFT, DEVICEMODE_BITLEN));
	//TODO: read preamble length, FreqHoppingPeriod, RegFifoTxBaseAddr, RegFifoAddrPtr, RegPayloadLength
	(void) SPI_handle->halSPIRead(REG_FIFOADDRPTR, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) SPI_handle->halSPIRead(REG_FIFOTXBASEADDR, 1, data);
	DB_PRINT(1, "Value in REG_FIFOTXBASEADDR is:%hu", data[0]);
	(void) SPI_handle->halSPIRead(REG_IRQFLAGS, 1, data);
	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);

	//rewrite the data TO put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value written in REG_OPMODE is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);
	//now write the configuration
	data[0] = SET_VAL(BANDWIDTH_500, BANDWIDTH_SHIFT, BANDWIDTH_BITLEN) |
			SET_VAL(CODERATE_1, CODRATE_SHIFT, CODRATE_BITLEN) |
			SET_VAL(EXPL_HEADER, IMPLHEADERMODEON_SHIFT, IMPLHEADERMODEON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG1 is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_MODEMCONFIG1, 1, data);
	//be careful since I will be writing 0 into the first two LSBits...
	data[0] = SET_VAL(SPREAD_FACTOR, SPREADINGFACTOR_SHIFT, SPREADINGFACTOR_BITLEN) |
			SET_VAL(TX_SINGLE_MODE, TXCONTINUOUSMODE_SHIFT, TXCONTINUOUSMODE_BITLEN) |
			SET_VAL(CRC_ENABLE, RXPAYLCRCON_SHIFT, RXPAYLCRCON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG2 is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_MODEMCONFIG2, 1, data);
	//write preamble length to 0x50
	data[0] = 0x50;
	(void) SPI_handle->halSPIWrite(REG_PREAMBLE_LSB, 1, data);\
	//set DIO0 to trigger interrupt when TXDone
		//DIO0-DIO1-DIO2-DIO3 (2bits per DIO)
	data[0] = 0b01000000;
	(void) SPI_handle->halSPIWrite(REG_DIOMAPPING1, 1, data);
	//TODO: Add validation checks.

	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraTransmit
 *
 * @brief      This function transmits the reqd payload data of specified length
 *
 * @param[in]  uint8_t * payload_data: buffer containing data to be TX
 *
 * @param[in]  uint8_t payload_len: length of data to be transmitted
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraTransmit(halLoraCurrentInfo_t * LoraCurrentInfo,
							uint8_t * payload_data, uint8_t payload_len)
{
	uint8_t data[2] = {0};
	if((payload_len == 0) || (payload_data == NULL) || (LoraCurrentInfo == NULL))
	{
		return halLoraInvalidArgs;
	}
	if((LoraCurrentInfo->isInitialized == false) || (LoraCurrentInfo->currentMode != halLoraModeTX))
	{
		return halLoraFail;
	}
	/*change mode to STANDBY!**/
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(STANDBY_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);
	//change the TX start address to 0 and reset address pointer here
	data[0] = 0x0;
	(void) SPI_handle->halSPIWrite(REG_FIFOTXBASEADDR, 1, data);
	(void) SPI_handle->halSPIWrite(REG_FIFOADDRPTR, 1, data);
	//load data byte by byte in a loop...hope this is okay
	DB_PRINT(1, "Now load the data into the buffer!!");
	for(int i = 0; i < payload_len; i++)
	{
		(void) SPI_handle->halSPIWrite(REG_FIFO, (uint8_t *) 1, &payload_data[i]);
	}
	//we write the payload length to be transferred
	(void) SPI_handle->halSPIWrite(REG_PAYLOADLENGTH, 1, &payload_len);
	//just to double check
	(void) SPI_handle->halSPIRead(REG_PAYLOADLENGTH, 1, data);
	DB_PRINT(1, "Value in REG_PAYLOADLENGTH is:%hu", data[0]);
	//read the TXAddr ptr...the offset should have happened ideally
	(void) SPI_handle->halSPIRead(REG_FIFOADDRPTR, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	DB_PRINT(1, "Now raise the TX BIT to transmit..");
	//loading should be complete. Now prepare for lift off!
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(TRANSMIT_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);
	//TODO: Add validation checks.
	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraConfigRX
 *
 * @brief      This function sets up all the required params to configure LORA as RX
 *
 * @param[in]  void
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraConfigRX(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	RegFrData regFreq = {0};
	//make sure OpMode is in sleep mode = 0
	(void) SPI_handle->halSPIRead(REG_OPMODE, 1, data);
	DB_PRINT(1, "Value in REG_OPMODE is:%hu", data[0]);
	DB_PRINT(1, "LongRangeMode:%hu, AccessSharedReg:%hu, LowfreqModeOn:%hu"
			"CurrentMode:%hu",
			GET_VAL(data[0], LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN),
			GET_VAL(data[0], ACCSHAREDREG_SHIFT, ACCSHAREDREG_BITLEN),
			GET_VAL(data[0], LOWFREQMODEON_SHIFT, LOWFREQMODEON_BITLEN),
			GET_VAL(data[0], DEVICEMODE_SHIFT, DEVICEMODE_BITLEN));
	//TODO: read preamble length, FreqHoppingPeriod, RegFifoTxBaseAddr, RegFifoAddrPtr, RegPayloadLength
	(void) SPI_handle->halSPIRead(REG_FIFOADDRPTR, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) SPI_handle->halSPIRead(REG_FIFORXBASEADDR, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXBASEADDR is:%hu", data[0]);
	(void) SPI_handle->halSPIRead(REG_IRQFLAGS, 1, data);
	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//read the preamble length
	SPI_handle->halSPIRead(REG_PREAMBLE_MSB, 2, data);
	DB_PRINT(1, "Value in REG_PREAMBLE_MSB: %hu, REG_PREAMBLE_LSB:%hu",
				data[0], data[1]);
	//TODO: Program this preamble length to max value?
	SPI_handle->halSPIRead(REG_RX_NB_BYTES, 2, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES: %hu", data[0]);
	//read the RegFifoRxByteAddr

	//rewrite the data TO put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value written in REG_OPMODE is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);
	//reset the address pointers here
	data[0] = 0x0;
	(void) SPI_handle->halSPIWrite(REG_FIFORXBASEADDR, 1, data);
	(void) SPI_handle->halSPIWrite(REG_FIFOADDRPTR, 1, data);
	//clear the IRQ flags
	data[0] = 0xFF;
	(void) SPI_handle->halSPIWrite(REG_IRQFLAGS, 1, data);
	//write preamble length to 0x50
	data[0] = 0x50;
	(void) SPI_handle->halSPIWrite(REG_PREAMBLE_LSB, 1, data);
	//play with the symb timeout
	data[0] = 0xFF; //this writes the LSB
	(void) SPI_handle->halSPIWrite(REG_SYMBTMO_LSB, 1, data);
	//set spreading factor as 7 and write MSB of SymbTimeout
	data[0] = SET_VAL(SPREAD_FACTOR, SPREADINGFACTOR_SHIFT, SPREADINGFACTOR_BITLEN) |
			SET_VAL(0x11, SYMBTMO_MSB_SHIFT, SYMBTMO_MSB_BITLEN); //this is the MSB
	// DB_PRINT(1, "New value written in REG_MODEMCONFIG2 is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_MODEMCONFIG2, 1, data);
	//now write the configuration
	data[0] = SET_VAL(BANDWIDTH_500, BANDWIDTH_SHIFT, BANDWIDTH_BITLEN) |
			SET_VAL(CODERATE_1, CODRATE_SHIFT, CODRATE_BITLEN) |
			SET_VAL(EXPL_HEADER, IMPLHEADERMODEON_SHIFT, IMPLHEADERMODEON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG1 is:%hu", data[0]);
	(void) SPI_handle->halSPIWrite(REG_MODEMCONFIG1, 1, data);
	//set DIO0 to trigger interrupt when RXDone (value 00 when RXDone)
		//DIO0-DIO1-DIO2-DIO3 (2bits per DIO)
	data[0] = 0b00000000;
	(void) SPI_handle->halSPIWrite(REG_DIOMAPPING1, 1, data);
	//TODO: Add validation checks.
	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraBeginReceiveMode
 *
 * @brief      This function starts the RX_CONTINUOUS operation
 *
 * @param[in]  halLoraCurrentInfo_t * LoraCurrentInfo: data structure of the Lora module state
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraBeginReceiveMode(halLoraCurrentInfo_t * LoraCurrentInfo)
{
	uint8_t data[2] = {0};
	if(LoraCurrentInfo == NULL)
	{
		return halLoraInvalidArgs;
	}
	if((LoraCurrentInfo->isInitialized == false) || (LoraCurrentInfo->currentMode != halLoraModeRX))
	{
		return halLoraFail;
	}
	//now start the receive operation
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(RX_CONT_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_OPMODE, 1, data);
	//TODO: Add validation checks
	return halLoraSuccess;

}

/*******************************************************************************
 * @fn         halLoraReadNumBytes
 *
 * @brief      This function gets the length of payload received
 *
 * @param[in]  uint8_t * numBytes: number of bytes to read from LORA
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraReadNumBytes(uint8_t * numBytes)
{
	uint8_t data[2] = {0};
	(void) SPI_handle->halSPIRead(REG_RX_NB_BYTES, 1, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES is:%hu", data[0]);
	memcpy(numBytes, data, 1); //copy 1 byte
	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraReadData
 *
 * @brief      This function gets new data from the RX buffer in LOR of specified length
 *
 * @param[in]  uint8_t * RX_buffer: buffer array provided from calling function
 *
 * @param[in] uint8_t payload_len: requested length of data
 *
 * @return     halLoraRet_t: status enums
 *
******************************************************************************/
halLoraRet_t halLoraReadData(uint8_t * RX_buffer, uint8_t payload_len, packetHealthInfo_t * packetHealthInfo)
{
	uint8_t data[2] = {0};
	uint8_t PacketRssi = 0;
	uint8_t PacketSNR = 0;
	if (payload_len == 0)
	{
		return halLoraInvalidArgs;
	}
	/*read the RSSI and SNR here?
	 * it has to be done in the period between header IRQ and RXDone IRQ*/
	(void) SPI_handle->halSPIRead(REG_PKTSNRVALUE, 1, data);
	PacketSNR = data[0];
	DB_PRINT(1, "Value in REG_PKTSNRVALUE is %d", PacketSNR);
	(void) SPI_handle->halSPIRead(REG_PKTRSSIVALUE, 1, data);
	PacketRssi = data[0];
	DB_PRINT(1, "Value in REG_PKTRSSIVALUE is %d", PacketRssi);
	calculateSignalHealth(PacketSNR, PacketRssi, packetHealthInfo);
	//clear these flags in the register
	data[0] = SET_VAL(1, IRQFLAG_RXDONE, IRQFLAG_BITLEN) |
			 SET_VAL(1, IRQFLAG_VALIDHEAD, IRQFLAG_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_IRQFLAGS, 1, data);
	//check if we have CRC.
	(void) SPI_handle->halSPIRead(REG_HOPCHANNEL, 1, data);
	if(GET_VAL(data[0], CRCONPAYLOAD_SHIFT, CRCONPAYLOAD_BITLEN) == CRC_DISABLE)
	{
		DB_PRINT(1,"****We don't have CRC at the end of the payload. Data integrity unchecked!!******");
	}
	//check if the CRC is good :)
	SPI_handle->halSPIRead(REG_IRQFLAGS, 1, data);
	//if, we have an error in CRC, clean buffer and exit
	if(GET_VAL(data[0],IRQFLAG_PAYLCRCERR,IRQFLAG_BITLEN) == 1)
	{
		data[0] = SET_VAL(1, IRQFLAG_PAYLCRCERR, IRQFLAG_BITLEN);
		(void) SPI_handle->halSPIWrite(REG_IRQFLAGS, 1, data);
		//for now just print data is bad
		DB_PRINT(1, "*******Received Data is BAD!!!****************");
	}
	//TODO: clean buffer after reading??
	uint8_t RX_curr_Address = 0;
	uint8_t FIFO_Address_Ptr = 0;
	(void) SPI_handle->halSPIRead(REG_FIFOADDRPTR, 1, data);
	FIFO_Address_Ptr = data[0];
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) SPI_handle->halSPIRead(REG_RX_NB_BYTES, 1, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES is:%hu", data[0]);
	if(payload_len > data[0])
	{
		//we cant read if requested data is more than what is present
		return halLoraInvalidArgs;
	}
	(void) SPI_handle->halSPIRead(REG_FIFORXCURRADDR, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXCURRADDR is:%hu", data[0]);
	RX_curr_Address = data[0];
	(void) SPI_handle->halSPIRead(REG_FIFORXBYTEADDR, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXBYTEADDR is:%hu", data[0]);
	//set the FifoAddrPtr to the RXCurrentAddress
	(void) SPI_handle->halSPIWrite(REG_FIFOADDRPTR, 1, &RX_curr_Address);
	for(int j = 0; j < payload_len; j++)
	{
		//read one byte at a time; and so allow the AddrPtr to increment
		(void) SPI_handle->halSPIRead(REG_FIFO, 1, (RX_buffer + j));
		DB_PRINT(1, "Data:%d - %c", j, RX_buffer[j]);
	}
	(void) SPI_handle->halSPIRead(REG_FIFOADDRPTR, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	return halLoraSuccess;
}

/*******************************************************************************
 * @fn         halLoraTXCompleteCB
 *
 * @brief      This is the CB function called inside Interrupt to disable the TX_DONE
 * 				interrupt from the LORA module
 *
 * @param[in]  void
 *
 * @return     void
 *
******************************************************************************/
void halLoraTXCompleteCB(void)
{
	uint8_t data[2] = {0};
	(void) SPI_handle->halSPIRead(REG_IRQFLAGS, 1, data);
//	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//clear this flag in the register
	data[0] = SET_VAL(1, IRQFLAG_TXDONE, IRQFLAG_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_IRQFLAGS, 1, data);
}

/*******************************************************************************
 * @fn         halLoraRXPayloadCB
 *
 * @brief      This is the CB function called inside Interrupt to disable the RX_DONE
 * 				interrupt from the LORA module
 *
 * @param[in]  void
 *
 * @return     void
 *
******************************************************************************/
void halLoraRXPayloadCB(void)
{
	uint8_t data[2] = {0};
	(void) SPI_handle->halSPIRead(REG_IRQFLAGS, 1, data);
//	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//clear these flags in the register
	data[0] = SET_VAL(1, IRQFLAG_RXDONE, IRQFLAG_BITLEN);
	(void) SPI_handle->halSPIWrite(REG_IRQFLAGS, 1, data);
}


static void calculateSignalHealth(uint8_t snrVal, uint8_t rssiVal, packetHealthInfo_t * packetHealthInfo)
{
	int8_t SNR_calc = ((int8_t ) snrVal)/4;
	packetHealthInfo->packetSNR = SNR_calc;
	packetHealthInfo->packetRSSI = -164 + rssiVal; //we are using lw freq here. So I am hardcoding
}
