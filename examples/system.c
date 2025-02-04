/*
 * system.c
 *
 *  Created on: Feb 3, 2025
 *      Author: yunus
 */

#include "system.h"
#include "main.h"
#include "INA219.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>


int8_t INA219_Read(void* intf, uint8_t reg, uint16_t *pRxData, uint8_t len)
{
   return HAL_I2C_Mem_Read((I2C_HandleTypeDef*)intf, (0x40 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pRxData, len, 100);
}


int8_t INA219_Write(void* intf, uint8_t reg, uint16_t *pTxData, uint8_t len)
{
    return HAL_I2C_Mem_Write((I2C_HandleTypeDef*)intf, (0x40 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pTxData, len, 100);
}


void setup()
{

	//Device configuration
	INA219_Device_t device1;
	device1 = INA219_NewDevice(1, 100.0, 3.0, &hi2c1,&INA219_Read, &INA219_Write);
	INA219_SetSettings(&device1, INA219_RANGE_16V, INA219_GAIN_320MV, INA219_ADC_10BIT, INA219_ADC_10BIT, INA219_MODE_SHUNT_BUS_CONTINUOUS);
	INA219_SetConfiguration(&device1);


	//Calibration
	INA219_CalibReg(&device1);

}


void loop()
{
	//Reads current and power
	INA219_ReadCurrentAmper(&device1);
	INA219_ReadPowerWatt(&device1);

}



