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
#define BITCLEAN(bitlen)		(~(0b11111111 << bitlen)) //if bitlen is 2, then output should be 0b00000011
#define LONGRANGEMODE_SHIFT		7
#define LONGRANGEMODE_BITLEN 	1
#define ACCSHAREDREG_SHIFT		6
#define ACCSHAREDREG_BITLEN		1
#define LOWFREQMODEON_SHIFT		3
#define LOWFREQMODEON_BITLEN 	1
#define DEVICEMODE_SHIFT        0
#define DEVICEMODE_BITLEN       3

#define IRQFLAG_BITLEN		1	//the length will always be 1
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
#define SYMBTMO_MSB_SHIFT		0
#define SYMBTMO_MSB_BITLEN		2

//LORA register addresses
#define REG_FIFO			0x00
#define REG_OPMODE			0x01
#define REG_MODEMCONFIG1	0x1D
#define REG_MODEMCONFIG2	0x1E
#define REG_FIFOADDRPTR		0x0D
#define REG_FIFOTXBASEADDR	0x0E
#define REG_FIFORXBASEADDR	0x0F
#define REG_FIFORXCURRADDR	0x10
#define REG_IRQFLAGS		0x12
#define REG_RX_NB_BYTES		0x13
#define REG_SYMBTMO_LSB		0x1F
#define REG_PREAMBLE_MSB	0x20
#define REG_PREAMBLE_LSB	0x21
#define REG_PAYLOADLENGTH	0x22
#define REG_FIFORXBYTEADDR	0x25
#define REG_DIOMAPPING1		0x40
#define REG_DIOMAPPING2		0x41

//data to write
#define FSK_OOK_MODE 	0
#define LORA_MODE		1
#define SLEEP_MODE		0b000
#define STANDBY_MODE	0b001
#define TRANSMIT_MODE	0b011
#define RX_SINGL_MODE	0b110
#define RX_CONT_MODE	0b101

//bandwidth
#define BANDWIDTH_1		0b0000 //7.8kHz
#define BANDWIDTH_2		0b0001 //104.kHz

//CR
#define CODERATE_1	0b001 // 4/5

//header
#define EXPL_HEADER	0
#define IMPL_HEADER	1

//TX continuous mode
#define TX_SINGLE_MODE	0
#define TX_CONT_MODE	1

//RX payload CRC
#define CRC_DISABLE	0
#define CRC_ENABLE	1

/**********************************************************************************************************************
* Macro section, defines with operation and function
*********************************************************************************************************************/
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
int32_t lora_test_transmit(void);
int32_t lora_test_receive(void);
void lora_TX_complete_cb(void);

#endif /* SX1278_H_ */
