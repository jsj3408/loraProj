/*
 * bmp280.c
 *
 *  Created on: Dec 18, 2022
 *      Author: johns
 */

/**********************************************************************************************************************
* Includes section
*********************************************************************************************************************/
#include "bmp280.h"
#include "time.h"
#include "math.h"

/**********************************************************************************************************************
* Local macro section, defines with operation and function
*********************************************************************************************************************/
/**********************************************************************************************************************
* Local typedef section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Local enum section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global variable
*********************************************************************************************************************/

uint16_t dig_T1 = 0;
int16_t dig_T2 = 0, dig_T3 = 0;
BMP280_S32_t t_fine = 0;
int32_t temp0 = 0;
int8_t rate = 0;
int32_t comp_temp;
clock_t finish;

/**********************************************************************************************************************
* Local function declaration section
*********************************************************************************************************************/

/**********************************************************************************************************************
* Global function definition
*********************************************************************************************************************/

uint8_t bmp280_init(void)
{
	uint8_t ret = 1;
	uint8_t retry = 10;
	uint8_t data = 0;
    uint8_t enablesettings = 0;
    /*
     * temp setting is in register 0xF4
     * bit 7-5: 001 (I only need 16bits of accuracy)
     * bit 4-2 : unused (turning off the pressure sensor)
     * bit 1-0 : 11 (normal mode)
     */
    uint8_t temp_setting = 0b00100011;
    /* config setting is written to register 0xF5
     * bit 7-5 :100 (500ms of standby)
     * bit 4-2 : 110 (filter coeff - 8)
     */
    uint8_t config_setting = 0b10011000;

    //Initialization should've been done

    //i2c_init();

	//read Chip identification number
	while (retry-- > 0 && data != BMP280_CHIPID)
	{
		i2c_transfer(BMP280_ADDR, kI2C_Read, 0xD0, &data, 1);
		printf("Data is 0x%x\n",data); //i am reading the chip identification number
	}
	if(data != BMP280_CHIPID)
	{
		ret = 0;
	}
	//next, write temperature settings and config settings
	retry = 10;
    while (retry-- > 0) //if chip identification number match
    {
    	i2c_transfer(BMP280_ADDR, kI2C_Write ,0xF4, &temp_setting, 1); //writing 16-bit temp resolution
		//i2c_transfer(kI2C_Read ,0xF4, &enablesettings, 1);
		printf("Write attempt : 0x%x vs read back 0x%x\n",temp_setting,enablesettings);
		i2c_transfer(BMP280_ADDR, kI2C_Write ,0xF5, &config_setting, 1);//writing configuration
    	//result = i2c_transfer(kI2C_Read ,0xFC, &temp, 1); //writing 16-bit temp resolution
    	//result = 1;
    }
	get_compensation_data();
	return ret;

}

void get_compensation_data(void)
{
	//uint8_t data_LSB = 0 , data_MSB = 0;
	//uint16_t data = 0;
	i2c_transfer(BMP280_ADDR, kI2C_Read ,0x88, &dig_T1, 2);
	//dig_T1 = ((data & 0xff00) >> 8) |((data & 0x00ff) << 8);
	printf("Param 1: 0x%x\n",dig_T1);
	//data = 0;
	i2c_transfer(BMP280_ADDR, kI2C_Read ,0x8A, &dig_T2, 2);
	//dig_T2 = ((data & 0xff00) >> 8) |((data & 0x00ff) << 8);
	printf("Param 2: 0x%x\n",dig_T2);
	//data = 0;
	i2c_transfer(BMP280_ADDR, kI2C_Read ,0x8C, &dig_T3, 2);
	//dig_T3 = ((data & 0xff00) >> 8) |((data & 0x00ff) << 8);
	printf("Param 3: 0x%x\n",dig_T3);
}

int32_t compensatedTemp(int32_t adcT)
{
	int32_t var1, var2;
	var1 = ((((adcT >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
	var2 = (((((adcT >> 4) - ((int32_t) dig_T1)) * ((adcT >> 4) - ((int32_t) dig_T1))) >> 12) *
	             ((int32_t) dig_T3)) >> 14;
	t_fine = var1 + var2;
	adcT = (t_fine * 5 + 128) >> 8;
	comp_temp = adcT ;
	return adcT;
}



int32_t tempRate(int32_t temp)
{
	static clock_t start = 0;
	if (!start)
		{
			start = clock();
		}
	static int32_t ret = 0;
	finish = clock();
	uint8_t diff = finish - start ;
	if (diff > 1) //if 1 second has elapsed, calculate the temperature rate
	{
		printf("Time elapsed: %d\n",diff);
		rate = (temp - temp0) / (finish - start);
		printf("Temp Rate: %d\n",rate);
		ret = rate;
		temp0 = temp;
		start = clock();
	}
	return ret;
}

state_t controlTempAlgorithm(int16_t setpoint, uint8_t ramp_rate)
{
	// lets first consider heating
	//first set limits. I have chosen (0-280)degC
	setpoint = (setpoint > 280) ? 280 : setpoint;
	setpoint = (setpoint < 0) ? 0 : setpoint;
	state_t ret = off;
	int32_t diffTemp = 0;

	int8_t ramp = 0;
	//handle floats
	int16_t curTemp = (int16_t) floor(((float) comp_temp) /100) ;
	diffTemp = abs(curTemp - setpoint) ;
	printf("Temp Difference: %d\n",diffTemp);
	ramp = (setpoint < curTemp) ? (- (int8_t) ramp_rate) : ramp_rate; //ramp value is negative if cooling, else same value

	if (diffTemp > 2*ramp_rate) // this is like an Intervention Offset value
	{	//if the current temperature rate is greater than the required ramp, turn off
		if (tempRate(curTemp) >= ramp)
		{
			ret = off;
		}
		else
		{
			ret = on;
		}
	}
	return ret;
}
