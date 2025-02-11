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

void serialPrint(const char *format, ...)
{
    char buffer[100];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    va_end(args);
}

int8_t i2cReadINA219(void* intf, uint8_t reg, uint8_t *pRxData, uint8_t len)
{
   return HAL_I2C_Mem_Read((I2C_HandleTypeDef*)intf, (0x40 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pRxData, len, 1000);
}

int8_t i2cWriteINA219(void* intf, uint8_t reg, uint8_t *pTxData, uint8_t len)
{
    return HAL_I2C_Mem_Write((I2C_HandleTypeDef*)intf, (0x40 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)pTxData, len, 1000);
}


INA219_Device_t device1;

void setup()
{

	// Config ayarı teker teker kullanıcı tarafından yapılmalı


	device1 = INA219_NewDevice(1, 0.1, 3.2, &hi2c3, &i2cReadINA219, &i2cWriteINA219);

	serialPrint("Setup init \n");
	INA219_Reset(&device1);

	//Device configuration
	uint16_t config;
	INA219_ReadRegister(&device1, 0x00, &config);
	serialPrint("First Config : %x\n", config);
	HAL_Delay(1000);


	INA219_SetBusAdcResolution(&device1, INA219_ADC_12BIT);
	INA219_SetShuntAdcResolution(&device1, INA219_ADC_12BIT);
	INA219_SetBusVoltage(&device1, INA219_BUSVOLTAGERANGE_16V);
	INA219_SetGain(&device1, INA219_GAIN_320MV);
	INA219_SetMode(&device1, INA219_MODE_SHUNT_BUS_CONTINUOUS);


	//Calibration
	serialPrint("Calibration \n");
	INA219_CalibReg(&device1);


	INA219_ReadRegister(&device1, 0x00, &config);
	serialPrint("Second config : %x\n", config);
	HAL_Delay(1000);

}


void loop()
{

	float bus_voltage;
	float shunt_voltage;
	float current;
	float power;

	INA219_ReadBusVoltage(&device1, &bus_voltage);
	INA219_ReadShuntVoltage(&device1, &shunt_voltage);
	INA219_ReadCurrentAmper(&device1, &current);
	INA219_ReadPowerWatt(&device1, &power);

	serialPrint("Bus Voltage : %.4f V \n", bus_voltage);
	serialPrint("Shunt Voltage : %.4f mV \n", shunt_voltage);
	serialPrint("Current : %.4f mA \n", current);
	serialPrint("Power : %.4f W \n ", power);
	serialPrint("--------------------------- \n");

	HAL_Delay(2000);


}



