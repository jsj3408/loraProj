/*
 * spi_comm.h
 *
 *  Created on: Feb 19, 2023
 *      Author: johns
 */

#ifndef SPI_COMM_H_
#define SPI_COMM_H_

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include <app_config.h>
#include "fsl_spi.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/


/**********************************************************************************************************************
* Typedef section
*********************************************************************************************************************/
typedef enum
{
	SPI_Write = 0,
	SPI_Read
}spi_direction_t;
/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
bool spi_init(void);
status_t spi_transfer(spi_direction_t spi_direction, uint8_t address,
						uint8_t * data, size_t dataSize, uint8_t * out);
int32_t spi_transfer_write(uint8_t address, int dataSize, uint8_t * buffer);
int32_t spi_transfer_read(uint8_t address, int dataSize, uint8_t * buffer);

#endif /* SPI_COMM_H_ */
