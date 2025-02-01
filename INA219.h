/*
 * INA219.h
 *
 *  Created on: Jan 27, 2025
 *      Author: yunus
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include <stdint.h>

typedef int8_t (*INA219_Read_t)(void* intf, uint8_t reg, uint16_t *pRxData, uint8_t len);
typedef int8_t (*INA219_Write_t)(void* intf, uint8_t reg, uint16_t *pTxData, uint8_t len);

typedef enum {
    INA219_RANGE_16V = 0, /* 16V range */
    INA219_RANGE_32V = 1  /* 32V range */
} BusVoltageRange_e;

typedef enum {
    INA219_GAIN_40MV = 0,
    INA219_GAIN_80MV = 1,
    INA219_GAIN_160MV = 2,
    INA219_GAIN_320MV = 3
} Gain_e;

typedef enum {
    INA219_ADC_9BIT = 0,
    INA219_ADC_10BIT = 1,
    INA219_ADC_11BIT = 2,
    INA219_ADC_12BIT = 3
} ADCResolution_e;

typedef enum {
    INA219_MODE_POWER_DOWN = 0,
    INA219_MODE_SHUNT_VOLTAGE_TRIGGERED = 1,
    INA219_MODE_BUS_VOLTAGE_TRIGGERED = 2,
    INA219_MODE_SHUNT_BUS_TRIGGERED = 3,
    INA219_MODE_ADC_OFF = 4,
    INA219_MODE_SHUNT_VOLTAGE_CONTINUOUS = 5,
    INA219_MODE_BUS_VOLTAGE_CONTINUOUS = 6,
    INA219_MODE_SHUNT_BUS_CONTINUOUS = 7
} OperatingMode_e;

typedef struct INA219_Settings {
    BusVoltageRange_e bus_voltage_range;   /* Bus voltage range: 16V or 32V */
    Gain_e gain;                           /* Gain settings: 40mV, 80mV, 160mV, 320mV */
    ADCResolution_e shunt_adc_resolution;  /* Shunt ADC resolution */
    ADCResolution_e bus_adc_resolution;    /* Bus ADC resolution */
    OperatingMode_e mode;                  /* Operating mode */
    float current_lsb;
    float power_lsb;
} INA219_Settings;

typedef struct INA219_Device_s {
    uint8_t chip_id;
    float r_shunt;
    float max_current;
    void* intf;         /* Interface (I2C) */
    INA219_Read_t read; /* Read function pointer */
    INA219_Write_t write; /* Write function pointer */
    INA219_Settings settings; /* Device settings */
} INA219_Device_t;


INA219_Device_t INA219_NewDevice(uint8_t chip_id, void *intf, INA219_Read_t read, INA219_Write_t write, INA219_Settings settings);
int16_t INA219_ReadRegister(INA219_Device_t *dev, uint8_t reg, int8_t *pRxBuffer, uint8_t len);
int16_t INA219_WriteRegister(INA219_Device_t *dev, uint8_t reg, int8_t *pTxBuffer, uint8_t len);
int16_t INA219_Read16BitRegister(INA219_Device_t *dev, uint8_t reg, int16_t *pRxBuffer);
void INA219_Write16BitRegister(INA219_Device_t *dev, uint8_t reg, int16_t value);
void INA219_SetBusVoltage(INA219_Device_t *dev, BusVoltageRange_e range);
void INA219_SetGain(INA219_Device_t *dev, Gain_e range);
void INA219_SetAdcResolution(INA219_Device_t *dev, ADCResolution_e range);
void INA219_SetMode(INA219_Device_t *dev, OperatingMode_e range);
void INA219_CalibReg(float r_shunt, float max_current);
void INA219_SetLsb(INA219_Device_t *dev);
void INA219_Reset(INA219_Device_t *dev);
void INA219_SetSettings(INA219_Device_t *dev, BusVoltageRange_e bus_voltage_range, Gain_e gain,
                        ADCResolution_e bus_adc_resolution, ADCResolution_e shunt_adc_resolution,
                        OperatingMode_e mode, float current_lsb, float power_lsb);
void INA219_SetConfiguration(INA219_Device_t *dev);
float INA219_CurrentAmper(INA219_Device_t *dev);
float INA219_PowerWatt(INA219_Device_t *dev);

#endif 
