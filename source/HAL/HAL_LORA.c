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

/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
halLoraRet_t halLoraInit(halLoraCurrentInfo_t * LoraCurrentInfo)
{
	uint8_t data[2] = {0};
	// put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value to write in REG_OPMODE is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);

	RegFrData regFreq = {0};
	//write the required carrier frequency into the LORA register
	lora_calculateRegFrequency(CARRIER_FREQ, CRYSTAL_OSC_FREQ, &regFreq);
	(void) spi_transfer(SPI_Write, REG_FR_MSB, &(regFreq.FrMSB), 1, NULL);
	(void) spi_transfer(SPI_Write, REG_FR_MID, &(regFreq.FrMID), 1, NULL);
	(void) spi_transfer(SPI_Write, REG_FR_LSB, &(regFreq.FrLSB), 1, NULL);
	//TODO: Add validation checks.
	LoraCurrentInfo->isInitialized = true;
	return halLoraSuccess;
}

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

halLoraRet_t halLoraConfigTX(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	//make sure OpMode is in sleep mode = 0
	(void) spi_transfer(SPI_Read, REG_OPMODE, NULL, 1, data);
	DB_PRINT(1, "Value in REG_OPMODE is:%hu", data[0]);
	DB_PRINT(1, "LongRangeMode:%hu, AccessSharedReg:%hu, LowfreqModeOn:%hu"
			"CurrentMode:%hu",
			GET_VAL(data[0], LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN),
			GET_VAL(data[0], ACCSHAREDREG_SHIFT, ACCSHAREDREG_BITLEN),
			GET_VAL(data[0], LOWFREQMODEON_SHIFT, LOWFREQMODEON_BITLEN),
			GET_VAL(data[0], DEVICEMODE_SHIFT, DEVICEMODE_BITLEN));
	//TODO: read preamble length, FreqHoppingPeriod, RegFifoTxBaseAddr, RegFifoAddrPtr, RegPayloadLength
	(void) spi_transfer(SPI_Read, REG_FIFOADDRPTR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) spi_transfer(SPI_Read, REG_FIFOTXBASEADDR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOTXBASEADDR is:%hu", data[0]);
	(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);

	//rewrite the data TO put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value written in REG_OPMODE is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//now write the configuration
	data[0] = SET_VAL(BANDWIDTH_1, BANDWIDTH_SHIFT, BANDWIDTH_BITLEN) |
			SET_VAL(CODERATE_1, CODRATE_SHIFT, CODRATE_BITLEN) |
			SET_VAL(EXPL_HEADER, IMPLHEADERMODEON_SHIFT, IMPLHEADERMODEON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG1 is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_MODEMCONFIG1, data, 1, NULL);
	//be careful since I will be writing 0 into the first two LSBits...
	data[0] = SET_VAL(7, SPREADINGFACTOR_SHIFT, SPREADINGFACTOR_BITLEN) |
			SET_VAL(TX_SINGLE_MODE, TXCONTINUOUSMODE_SHIFT, TXCONTINUOUSMODE_BITLEN) |
			SET_VAL(CRC_DISABLE, RXPAYLCRCON_SHIFT, RXPAYLCRCON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG2 is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_MODEMCONFIG2, data, 1, NULL);
	//write preamble length to 0x50
	data[0] = 0x50;
	(void) spi_transfer(SPI_Write, REG_PREAMBLE_LSB, data, 1, NULL);\
	//set DIO0 to trigger interrupt when TXDone
		//DIO0-DIO1-DIO2-DIO3 (2bits per DIO)
	data[0] = 0b01000000;
	(void) spi_transfer(SPI_Write, REG_DIOMAPPING1, data, 1, NULL);
	//TODO: Add validation checks.

	return halLoraSuccess;
}

halLoraRet_t halLoraTransmit(uint8_t * payload_data, uint8_t payload_len)
{
	uint8_t data[2] = {0};
	if((payload_len == 0) || (payload_data == NULL))
	{
		return halLoraInvalidArgs;
	}
	/*change mode to STANDBY!**/
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(STANDBY_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//change the TX start address to 0 and reset address pointer here
	data[0] = 0x0;
	(void) spi_transfer(SPI_Write, REG_FIFOTXBASEADDR, data, 1, NULL);
	(void) spi_transfer(SPI_Write, REG_FIFOADDRPTR, data, 1, NULL);
	//load data byte by byte in a loop...hope this is okay
	DB_PRINT(1, "Now load the data into the buffer!!");
	for(int i = 0; i < payload_len; i++)
	{
		spi_transfer(SPI_Write, REG_FIFO, (uint8_t *) &payload_data[i], 1, NULL);
	}
	//we write the payload length to be transferred
	(void) spi_transfer(SPI_Write, REG_PAYLOADLENGTH, &payload_len, 1, NULL);
	//just to double check
	(void) spi_transfer(SPI_Read, REG_PAYLOADLENGTH, NULL, 1, data);
	DB_PRINT(1, "Value in REG_PAYLOADLENGTH is:%hu", data[0]);
	//read the TXAddr ptr...the offset should have happened ideally
	(void) spi_transfer(SPI_Read, REG_FIFOADDRPTR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	DB_PRINT(1, "Now raise the TX BIT to transmit..");
	//loading should be complete. Now prepare for lift off!
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(TRANSMIT_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//TODO: Add validation checks.
	return halLoraSuccess;
}

halLoraRet_t halLoraConfigRX(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	RegFrData regFreq = {0};
	//make sure OpMode is in sleep mode = 0
	(void) spi_transfer(SPI_Read, REG_OPMODE, NULL, 1, data);
	DB_PRINT(1, "Value in REG_OPMODE is:%hu", data[0]);
	DB_PRINT(1, "LongRangeMode:%hu, AccessSharedReg:%hu, LowfreqModeOn:%hu"
			"CurrentMode:%hu",
			GET_VAL(data[0], LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN),
			GET_VAL(data[0], ACCSHAREDREG_SHIFT, ACCSHAREDREG_BITLEN),
			GET_VAL(data[0], LOWFREQMODEON_SHIFT, LOWFREQMODEON_BITLEN),
			GET_VAL(data[0], DEVICEMODE_SHIFT, DEVICEMODE_BITLEN));
	//TODO: read preamble length, FreqHoppingPeriod, RegFifoTxBaseAddr, RegFifoAddrPtr, RegPayloadLength
	(void) spi_transfer(SPI_Read, REG_FIFOADDRPTR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) spi_transfer(SPI_Read, REG_FIFORXBASEADDR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXBASEADDR is:%hu", data[0]);
	(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//read the preamble length
	spi_transfer(SPI_Read, REG_PREAMBLE_MSB, NULL, 2, data);
	DB_PRINT(1, "Value in REG_PREAMBLE_MSB: %hu, REG_PREAMBLE_LSB:%hu",
				data[0], data[1]);
	//TODO: Program this preamble length to max value?
	spi_transfer(SPI_Read, REG_RX_NB_BYTES, NULL, 2, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES: %hu", data[0]);
	//read the RegFifoRxByteAddr

	//rewrite the data TO put OpMode to Sleep and to LORA mode
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(SLEEP_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	DB_PRINT(1, "New value written in REG_OPMODE is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//reset the address pointers here
	data[0] = 0x0;
	(void) spi_transfer(SPI_Write, REG_FIFORXBASEADDR, data, 1, NULL);
	(void) spi_transfer(SPI_Write, REG_FIFOADDRPTR, data, 1, NULL);
	//clear the IRQ flags
	data[0] = 0xFF;
	(void) spi_transfer(SPI_Write, REG_IRQFLAGS, data, 1, NULL);
	//write preamble length to 0x50
	data[0] = 0x50;
	(void) spi_transfer(SPI_Write, REG_PREAMBLE_LSB, data, 1, NULL);
	//play with the symb timeout
	data[0] = 0xFF; //this writes the LSB
	(void) spi_transfer(SPI_Write, REG_SYMBTMO_LSB, data, 1, NULL);
	//set spreading factor as 7 and write MSB of SymbTimeout
	data[0] = SET_VAL(7, SPREADINGFACTOR_SHIFT, SPREADINGFACTOR_BITLEN) |
			SET_VAL(0x11, SYMBTMO_MSB_SHIFT, SYMBTMO_MSB_BITLEN); //this is the MSB
	// DB_PRINT(1, "New value written in REG_MODEMCONFIG2 is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_MODEMCONFIG2, data, 1, NULL);
	//now write the configuration
	data[0] = SET_VAL(BANDWIDTH_1, BANDWIDTH_SHIFT, BANDWIDTH_BITLEN) |
			SET_VAL(CODERATE_1, CODRATE_SHIFT, CODRATE_BITLEN) |
			SET_VAL(EXPL_HEADER, IMPLHEADERMODEON_SHIFT, IMPLHEADERMODEON_BITLEN);
	DB_PRINT(1, "New value written in REG_MODEMCONFIG1 is:%hu", data[0]);
	(void) spi_transfer(SPI_Write, REG_MODEMCONFIG1, data, 1, NULL);
	//set DIO0 to trigger interrupt when RXDone (value 00 when RXDone)
		//DIO0-DIO1-DIO2-DIO3 (2bits per DIO)
	data[0] = 0b00000000;
	(void) spi_transfer(SPI_Write, REG_DIOMAPPING1, data, 1, NULL);
	//TODO: Add validation checks.
	return halLoraSuccess;
}

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
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//TODO: Add validation checks
	return halLoraSuccess;

}
halLoraRet_t halLoraReadNumBytes(uint8_t * numBytes)
{
	uint8_t data[2] = {0};
	(void) spi_transfer(SPI_Read, REG_RX_NB_BYTES, NULL, 1, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES is:%hu", data[0]);
	memcpy(numBytes, data, 1); //copy 1 byte
	return halLoraSuccess;
}

halLoraRet_t halLoraReadData(uint8_t * RX_buffer, uint8_t payload_len)
{
	if (payload_len > 256)
	{
		return halLoraInvalidArgs;
	}
	//TODO: clean buffer after reading??
	uint8_t data[2] = {0};
	uint8_t RX_curr_Address = 0;
	(void) spi_transfer(SPI_Read, REG_FIFOADDRPTR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	(void) spi_transfer(SPI_Read, REG_RX_NB_BYTES, NULL, 1, data);
	DB_PRINT(1, "Value in REG_RX_NB_BYTES is:%hu", data[0]);
	if(payload_len > data[0])
	{
		//we cant read if requested data is more than what is present
		return halLoraInvalidArgs;
	}
	(void) spi_transfer(SPI_Read, REG_FIFORXCURRADDR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXCURRADDR is:%hu", data[0]);
	RX_curr_Address = data[0];
	(void) spi_transfer(SPI_Read, REG_FIFORXBYTEADDR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFORXBYTEADDR is:%hu", data[0]);
	//set the FifoAddrPtr to the RXCurrentAddress
	(void) spi_transfer(SPI_Write, REG_FIFOADDRPTR, &RX_curr_Address, 1, NULL);
	for(int j = 0; j < payload_len; j++)
	{
		//read one byte at a time; and so allow the AddrPtr to increment
		(void) spi_transfer(SPI_Read, REG_FIFO, NULL, 1, (RX_buffer + j));
		DB_PRINT(1, "Data:%d - %c", j, RX_buffer[j]);
	}
	return halLoraSuccess;
}

void halLoraTXCompleteCB(void)
{
	uint8_t data[2] = {0};
	(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
//	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//clear this flag in the register
	data[0] = SET_VAL(1, IRQFLAG_TXDONE, IRQFLAG_BITLEN);
	(void) spi_transfer(SPI_Write, REG_IRQFLAGS, data, 1, NULL);
}

void halLoraRXPayloadCB(void)
{
	uint8_t data[2] = {0};
	(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
//	DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
	//clear these flags in the register
	data[0] = SET_VAL(1, IRQFLAG_RXDONE, IRQFLAG_BITLEN);
	(void) spi_transfer(SPI_Write, REG_IRQFLAGS, data, 1, NULL);
}
