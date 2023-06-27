/*
 * sx1278.c
 *
 *  Created on: Apr 22, 2023
 *      Author: johns
 */

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#pragma once
#include "sx1278.h"
#include "spi_comm.h"
#include "app_config.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
#define READ_REGS	0
#define WRITE_REGS	0

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
int32_t lora_init(void)
{
	//clear the GPIO light pins
	return 1;
}


int32_t lora_test_transmit(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	char test_payload[] = "This is a Test Payload.";
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

	/*change mode to STANDBY!**/
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(STANDBY_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//change the TX start address to 0 and reset address pointer here
	data[0] = 0x0;
	(void) spi_transfer(SPI_Write, REG_FIFOTXBASEADDR, data, 1, NULL);
	(void) spi_transfer(SPI_Write, REG_FIFOADDRPTR, data, 1, NULL);
	//load data byte by byte in a loop...hope this is okay
	//this payload should exclude null character
	DB_PRINT(1, "Now load the data into the buffer!!");
	for(int i = 0; i < (sizeof(test_payload) - 1); i++)
	{
		spi_transfer(SPI_Write, REG_FIFO, (uint8_t *) &test_payload[i], 1, NULL);
	}
	//read the TXAddr ptr...the offset should have happened ideally
	(void) spi_transfer(SPI_Read, REG_FIFOADDRPTR, NULL, 1, data);
	DB_PRINT(1, "Value in REG_FIFOADDRPTR is:%hu", data[0]);
	DB_PRINT(1, "Now raise the TX BIT to transmit..");
	//loading should be complete. Now prepare for lift off!
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(TRANSMIT_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//now wait for the TX_interrupt to be issued
	data[0] = 0;
	volatile int retry = 50;
	GPIO_ClearPinsOutput(GPIOB, 0x01 << 18);
	GPIO_ClearPinsOutput(GPIOB, 0x01 << 19);
	while((retry-- > 0) && ((data[0] & (0b00001000)) != 0b00001000))
	{
		data[0] = 0;
		(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
		DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
		SysTick_DelayTicks(100U);
	}
	GPIO_SetPinsOutput(GPIOB, 0x01 << 18);
	GPIO_SetPinsOutput(GPIOB, 0x01 << 19);
	if(retry < 1)
	{
		DB_PRINT(1, "Retries failed and no TX related interrupt has been raised!");
		GPIO_ClearPinsOutput(GPIOB, 0x01 << 18);
	}
	else
	{
		DB_PRINT(1, "If you are here the TX Done interrupt has been raised!");
		GPIO_ClearPinsOutput(GPIOB, 0x01 << 19);
	}

	//TODO: Configure preamble length later (in TX and RX) and check what this is. Write IRQ mask also

//	//read back the data to verify
//	(void) spi_transfer(SPI_Read, REG_OPMODE, NULL, 1, data);
//		DB_PRINT(1, "LongRangeMode:%hu, AccessSharedReg:%hu, LowfreqModeOn:%hu,"
//				" CurrentMode:%hu",
//				GET_VAL(data[0], LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN),
//				GET_VAL(data[0], ACCSHAREDREG_SHIFT, ACCSHAREDREG_BITLEN),
//				GET_VAL(data[0], LOWFREQMODEON_SHIFT, LOWFREQMODEON_BITLEN),
//				GET_VAL(data[0], DEVICEMODE_SHIFT, DEVICEMODE_BITLEN));

	return 1;
}

int32_t lora_test_receive(void)
{
	//each SPI read operation requires two bytes of output (put for safety)
	uint8_t data[2] = {0};
	uint8_t RX_interrupt_val = 0;
	char test_payload[] = "This is a Test Payload.";
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
	//write preamble length to 0xFF
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

	//now start the receive operation
	data[0] = SET_VAL(LORA_MODE, LONGRANGEMODE_SHIFT, LONGRANGEMODE_BITLEN) |
				SET_VAL(RX_CONT_MODE, DEVICEMODE_SHIFT, DEVICEMODE_BITLEN);
	(void) spi_transfer(SPI_Write, REG_OPMODE, data, 1, NULL);
	//now wait for the RX_interrupt to be issued
	data[0] = 0;
	RX_interrupt_val = SET_VAL(1, IRQFLAG_RXTIMEOUT, IRQFLAG_BITLEN) |
			SET_VAL(1, IRQFLAG_RXDONE, IRQFLAG_BITLEN) |
			SET_VAL(1, IRQFLAG_VALIDHEAD, IRQFLAG_BITLEN);
	volatile int retry = 50;
	GPIO_ClearPinsOutput(GPIOB, 0x01 << 18);
	GPIO_ClearPinsOutput(GPIOB, 0x01 << 19);
	while((retry-- > 0) && ((data[0] & RX_interrupt_val) == 0)) //loop as long as we dont have any of these bits set
	{
		data[0] = 0;
		(void) spi_transfer(SPI_Read, REG_IRQFLAGS, NULL, 1, data);
		DB_PRINT(1, "Value in REG_IRQFLAGS is:%hu", data[0]);
		SysTick_DelayTicks(25U);
	}
	GPIO_SetPinsOutput(GPIOB, 0x01 << 18);
	GPIO_SetPinsOutput(GPIOB, 0x01 << 19);
	if(retry < 1)
	{
		DB_PRINT(1, "Retries failed and no RX related interrupt has been raised!");
		GPIO_ClearPinsOutput(GPIOB, 0x01 << 18);
	}
	else
	{
		DB_PRINT(1, "Some RX related interrupt has been raised!");
		GPIO_ClearPinsOutput(GPIOB, 0x01 << 19);
	}
}
