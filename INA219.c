/*
 * INA219.c
 *
 *  Created on: Jan 27, 2025
 *      Author: yunus
 */

#include "INA219.h"

// INA219 Register Addresses
#define REG_CONFIG 0x00
#define REG_SHUNTVOLTAGE 0x01
#define REG_BUSVOLTAGE 0x02
#define REG_POWER 0x03
#define REG_CURRENT 0x04
#define REG_CALIBRATION 0x05

// Initializes a new INA219 device structure
INA219_Device_t INA219_NewDevice(uint8_t chip_id, void *intf, INA219_Read_t read, INA219_Write_t write, INA219_Settings settings)
{
    INA219_Device_t dev;
    dev.chip_id = chip_id;
    dev.intf = intf;
    dev.read = read;
    dev.write = write;
    dev.settings = settings;

    return dev;
}

// Sets the device configuration settings
void INA219_SetSettings(INA219_Device_t *dev, BusVoltageRange_e bus_voltage_range, Gain_e gain,
                        ADCResolution_e bus_adc_resolution, ADCResolution_e shunt_adc_resolution,
                        OperatingMode_e mode, float current_lsb, float power_lsb)
{
    dev->settings.bus_voltage_range = bus_voltage_range;
    dev->settings.gain = gain;
    dev->settings.bus_adc_resolution = bus_adc_resolution;
    dev->settings.shunt_adc_resolution = shunt_adc_resolution;
    dev->settings.mode = mode;
    dev->settings.current_lsb = current_lsb;
    dev->settings.power_lsb = power_lsb;
}

// Applies the settings to the device
void INA219_SetConfiguration(INA219_Device_t *dev)
{
    INA219_SetBusVoltage(dev, dev->settings.bus_voltage_range);
    INA219_SetGain(dev, dev->settings.gain);
    INA219_SetBusAdcResolution(dev, dev->settings.bus_adc_resolution);
    INA219_SetShuntAdcResolution(dev, dev->settings.shunt_adc_resolution);
    INA219_SetMode(dev, dev->settings.mode);
}


// Reads a register from the device
int16_t INA219_ReadRegister(INA219_Device_t *dev, uint8_t reg, int8_t *pRxBuffer, uint8_t len)
{
    return dev->read(dev->intf, reg, pRxBuffer, len);
}

// Writes to a register in the device
int16_t INA219_WriteRegister(INA219_Device_t *dev, uint8_t reg, int8_t *pTxBuffer, uint8_t len)
{
    return dev->write(dev->intf, reg, pTxBuffer, len);
}

// Sets the bus voltage range
void INA219_SetBusVoltage(INA219_Device_t *dev, BusVoltageRange_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, (uint8_t*)&buffer, 2);

    buffer &= 0b1101111111111111;
    buffer |= (uint16_t)range << 13;
    dev->settings.bus_voltage_range = range;

    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer, 2);
}

// Sets the gain for shunt voltage measurement
void INA219_SetGain(INA219_Device_t *dev, Gain_e range)
{
    uint16_t reg = 0;
    INA219_ReadRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);

    buffer &= 0b1110011111111111;
    buffer |= (uint16_t)range << 11;
    dev->settings.gain = range;

    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);
}

// Sets the ADC resolution for bus voltage
void INA219_SetBusAdcResolution(INA219_Device_t *dev, ADCResolution_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG,(uint8_t*)&buffer,2);

    reg &= 0b1111100001111111;
    reg |= (uint16_t)range << 7;
    dev->settings.bus_adc_resolution = range;

    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);

}

// Sets the ADC resolution for shunt voltage
void INA219_SetShuntAdcResolution(INA219_Device_t *dev, ADCResolution_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG,(uint8_t*)&buffer,2);

    reg &= 0b1111111110000111;
    reg |= (uint16_t)range << 3;
    dev->settings.shunt_adc_resolution = range;

    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);
}



// Sets the device operating mode
void INA219_SetMode(INA219_Device_t *dev, OperatingMode_e range)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG,(uint8_t*)&buffer,2);

    reg &= 0b1111111111111000;
    reg |= (uint16_t)range;
    dev->settings.mode = range;

    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);
}


// Calculates and sets LSB values
void INA219_SetLsb(INA219_Device_t *dev)
{
    dev->settings.current_lsb = dev->max_current / 32768.0;
    dev->settings.power_lsb = 20 * dev->settings.current_lsb;
}


// Writes the calibration register
void INA219_CalibReg(INA219_Device_t *dev)
{
    float current_lbs = dev->max_current / 32768.0;
    uint16_t calib = (uint16_t)(0.04096 / (current_lbs * dev->r_shunt));
    INA219_WriteRegister(dev, REG_CALIBRATION, (uint8_t*)&calib,2);

}


// Reads and calculates current in amperes
float INA219_CurrentAmper(INA219_Device_t *dev)
{
    int16_t current_register = 0;
    INA219_ReadRegister(dev, REG_CURRENT, (uint8_t*)&current_register,2);
    return (float)current_register * dev->settings.current_lsb;
}


// Reads and calculates power in watts
float INA219_PowerWatt(INA219_Device_t *dev)
{
    int16_t power_register = 0;
    INA219_ReadRegister(dev, REG_POWER, (uint8_t*)&power_register,2);
    return (float)power_register * dev->settings.power_lsb;
}


// Resets the INA219 device
void INA219_Reset(INA219_Device_t *dev)
{
    uint16_t buffer = 0;
    INA219_ReadRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);
    reg |= 0x8000;
    INA219_WriteRegister(dev, REG_CONFIG, (uint8_t*)&buffer,2);
}
