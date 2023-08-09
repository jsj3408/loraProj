/*
 * spi_comm.c
 *
 *  Created on: Feb 19, 2023
 *      Author: johns
 */

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "spi_comm.h"

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/
//spi_master_handle_t spiHandle;
static spi_master_config_t masterConfig;


/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/
/*******************************************************************************
 * @fn         spi_init
 *
 * @brief      This initializes the SPI module inside the FRDM-KL25
 *
 * @param[in]  void
 *
 * @return     bool: true if success else fail
 *
******************************************************************************/
bool spi_init(void)
{
    spi_master_config_t masterConfig = {0};
    spi_slave_config_t slaveConfig = {0};
    spi_transfer_t xfer = {0};
    uint32_t sourceClock = 0U;

	bool ret = true;
	//SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 500000U;
	masterConfig.direction = kSPI_MsbFirst;
	masterConfig.enableMaster = true;
	masterConfig.enableStopInWaitMode = false;
	masterConfig.polarity = kSPI_ClockPolarityActiveHigh;
	masterConfig.phase = kSPI_ClockPhaseFirstEdge;
	masterConfig.pinMode = kSPI_PinModeNormal;
	masterConfig.outputMode = kSPI_SlaveSelectAsGpio;
	sourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
	SPI_MasterInit(SPI0, &masterConfig, sourceClock);
	return ret;
}

/*******************************************************************************
 * @fn         spi_transfer
 *
 * @brief      This API is used to send and receive packets via the board's SPI interface.
 *
 * @param[in]  spi_direction_t spi_direction: read or write
 *
 * @param[in]  uint8_t address: the register address to read from/write to
 *
 * @param[in]  uint8_t * data : this is the data that we want to write into the address.
 * 				NULL during a read
 *
 * @param[in]  size_t dataSize: the size of the data to write/read (in bytes)
 *
 * @param[in]  uint8_t * out : the buffer to be able to store the data being read of size
 * 				dataSize. Put to NULL during a write
 *
 * @return     status_t: generic status return codes by NXP of enum type _generic_status
 *
******************************************************************************/
status_t spi_transfer(spi_direction_t spi_direction, uint8_t address,
						uint8_t * data, size_t dataSize, uint8_t * out)
{
	status_t ret = kStatus_Success;
	spi_transfer_t xfer_address;
	spi_transfer_t xfer_data;
	//this should be 0x7F if write operation or 0xFF if read operation
	uint8_t bit_mod = (spi_direction == SPI_Write) ? 0x80 : 0x00;
	//here we set the MSBit to 1 or 0 based on read/write
	address |= bit_mod;

	//set up the address struct.
	xfer_address.txData = &address;
	xfer_address.rxData = NULL;
	xfer_address.dataSize = 1;

	//set up the data struct whose data needs to be written
	xfer_data.txData = data;
	xfer_data.rxData = out;
	xfer_data.dataSize = dataSize;
	//Set the CS(SS pin) to Low (because we have configured this pin as a GPIo than SPI)
	GPIO_ClearPinsOutput(GPIOD, 0x01);
	//first send the address (control byte)
	ret = SPI_MasterTransferBlocking(SPI0, &xfer_address);
	//then the struct to send/receive data
	ret = SPI_MasterTransferBlocking(SPI0, &xfer_data);
	//Raise back the CS/SS to high
	GPIO_SetPinsOutput(GPIOD, 0x01);

	return ret;
}


/*******************************************************************************
 * @fn         spi_transfer_read
 *
 * @brief      This API is used to receive packets via the board's SPI interface.
 *
 * @param[in]  uint8_t address: the register address to read from
 *
 * @param[in]  size_t dataSize: the size of the data to read (in bytes)
 *
 * @param[in]  uint8_t * buffer : the buffer to be able to store the data being read of size
 * 				int.
 *
 * @return     int32_t: generic status return codes by NXP of enum type _generic_status
 *
******************************************************************************/
int32_t spi_transfer_read(uint8_t address, int dataSize, uint8_t * buffer)
{
	return spi_transfer(SPI_Read, address, NULL, (size_t) dataSize, buffer);
}

/*******************************************************************************
 * @fn         spi_transfer_write
 *
 * @brief      This API is used to send and receive packets via the board's SPI interface.
 *
 * @param[in]  spi_direction_t spi_direction: read or write
 *
 * @param[in]  uint8_t address: the register address to write to
 *
 * @param[in]  uint8_t * buffer : this is the data that we want to write into the address.
 * 				NULL during a read
 *
 * @param[in]  int dataSize: the size of the data to write/read (in bytes)
 *
 * @return     int32_t: generic status return codes by NXP of enum type _generic_status
 *
******************************************************************************/
int32_t spi_transfer_write(uint8_t address, int dataSize, uint8_t * buffer)
{
	return spi_transfer(SPI_Write, address, buffer, (size_t) dataSize, NULL);
}
