/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "i2c_comm.h"

/**********************************************************************************************************************
* Defines section
*********************************************************************************************************************/
#define BMP280_ADDR 0x76
#define BMP280_CHIPID 0x58
#define BMP280_S32_t long signed int
#define BMP280_U32_t long unsigned int
#define BMP280_S64_t long long signed int

/**********************************************************************************************************************
* Enum section
*********************************************************************************************************************/

typedef enum{
	off = 0,
	on
} state_t;


/**********************************************************************************************************************
* Function declaration section
*********************************************************************************************************************/
uint8_t bmp280_init(void);
void get_compensation_data(void);
int32_t compensatedTemp(int32_t adcT);
int32_t tempRate(int32_t temp);
state_t controlTempAlgorithm(int16_t setpoint, uint8_t ramp_rate);
