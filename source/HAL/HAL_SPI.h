/*
 * HAL_SPI.h
 *
 *  Created on: Aug 9, 2023
 *      Author: johns
 */

#ifndef HAL_HAL_SPI_H_
#define HAL_HAL_SPI_H_

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "app_config.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/


/**********************************************************************************************************************
* Typedef section
*********************************************************************************************************************/
typedef enum
{
	halSPIInvalidArgs = -2,
	halSPIFail,
	halSPISuccess,
}halSPIRet_t;

typedef bool (* SPIInit)(void);
typedef int32_t (* SPIRead)(uint8_t address, int dataSize, uint8_t * buffer);
typedef int32_t (* SPIWrite)(uint8_t address, int dataSize, uint8_t * buffer);
typedef bool (* SPIDeInit)(void);

typedef struct
{
	SPIInit halSPIInit;
	SPIRead halSPIRead;
	SPIWrite halSPIWrite;
	SPIDeInit halSPIDeInit;
}halSPIAction_t;
/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
halSPIAction_t * halConfigureSPIHandle(SPIInit SPI_init_fn, SPIRead SPI_read_fn,
									SPIWrite SPI_write_fn, SPIDeInit SPI_Deinit_fn);
#endif /* HAL_HAL_SPI_H_ */
