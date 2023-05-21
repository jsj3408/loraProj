/*
 * sx1278.h
 *
 *  Created on: Apr 22, 2023
 *      Author: johns
 */

#ifndef SX1278_H_
#define SX1278_H_

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "spi_comm.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Macro section, defines with operation and function
*********************************************************************************************************************/
#define BITCLEAN(bitlen)		(~(0b11111111 << bitlen)) //if bitlen is 2, then output should be 0b00000011
#define LONGRANGEMODE_SHIFT		7
#define LONGRANGEMODE_BITLEN 	1
#define ACCSHAREDREG_SHIFT		6
#define ACCSHAREDREG_BITLEN		1
#define LOWFREQMODEON_SHIFT		3
#define LOWFREQMODEON_BITLEN 	1
#define DEVICEMODE_SHIFT        0
#define DEVICEMODE_BITLEN       3

#define IRQFLAG_RXTIMEOUT	7
#define IRQFLAG_RXDONE		6
#define IRQFLAG_PAYLCRCERR	5
#define IRQFLAG_VALIDHEAD	4
#define IRQFLAG_TXDONE		3
#define IRQFLAG_CADDONE		2
#define IRQFLAG_FHSSCHANGECH	1
#define IRQFLAG_CADDETECT	0

#define BANDWIDTH_SHIFT		4
#define BANDWIDTH_BITLEN	4
#define CODRATE_SHIFT		1
#define CODRATE_BITLEN		3
#define IMPLHEADERMODEON_SHIFT	0
#define IMPLHEADERMODEON_BITLEN	1
#define SPREADINGFACTOR_SHIFT	4
#define SPREADINGFACTOR_BITLEN	4
#define TXCONTINUOUSMODE_SHIFT	3
#define TXCONTINUOUSMODE_BITLEN	1
#define RXPAYLCRCON_SHIFT		2
#define RXPAYLCRCON_BITLEN		1


#define GET_VAL(x,SHIFT,BITLEN) 	((x >> SHIFT) & BITCLEAN(BITLEN)) //x is the input uint8_t value
#define SET_VAL(x,SHIFT, BITLEN) 	((x & BITCLEAN(BITLEN)) << SHIFT)   //x stands for the bit data to be written
#define CLEAR_VAL(x,SHIFT,BITLEN)	(x & (~BITCLEAN(BITLEN) << SHIFT))  //x is the input uint8_t value to be cleared

/**********************************************************************************************************************
* Typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Enum section
*********************************************************************************************************************/


/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
int32_t lora_init(void);

#endif /* SX1278_H_ */
