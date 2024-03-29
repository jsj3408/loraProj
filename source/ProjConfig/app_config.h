/*
 * app_config.h
 *
 *  Created on: Feb 25, 2023
 *      Author: johns
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_clock.h"

#include "fsl_port.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
//#define BAREMETAL
#ifdef BAREMETAL
#define LORA_TX
//#define LORA_RX

#else

#define APP_TX
//#define APP_RX

#ifdef APP_RX
#define USE_DISPLAY
#endif

#endif
/**********************************************************************************************************************
* Macro section, defines with operation and function
*********************************************************************************************************************/
#define DB_PRINT(ENABLE, print, ...) \
	{if (ENABLE == 1) \
		{\
			printf(print, ##__VA_ARGS__);\
			printf("\n");\
		}\
	}


/**********************************************************************************************************************
* Typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Enum section
*********************************************************************************************************************/


/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
void SysTick_DelayTicks(uint32_t n);
void SysTick_Handler(void);
#endif /* APP_CONFIG_H_ */
