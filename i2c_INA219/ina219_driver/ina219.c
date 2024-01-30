#include "ina219.h"

static int INA219_ReadRegister(INA219_t *my_INA, uint8_t const reg)
{
//    HAL_StatusTypeDef code = HAL_ERROR;
//    code = HAL_I2C_Mem_Read(my_INA->I2C_handle, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, my_INA->receive, 2, HAL_MAX_DELAY);
    return 1;
}

static int INA219_WriteRegister(INA219_t *my_INA, uint8_t const reg, uint16_t data)
{
    my_INA->send[0] = (data >> 8) & 0xff;  // upper byte
    my_INA->send[1] = (data >> 0) & 0xff;
//    return HAL_I2C_Mem_Write(my_INA->I2C_handle, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, my_INA->send, 2, HAL_MAX_DELAY);
    return 1;
}


static void setCalibration_16V_400mA(INA219_t *my_INA)
{

    uint16_t config           =   SHUNT_BUS_CONT | SADC_12BIT_532US
                       | BADC_12BIT_532US | PG_GAIN_1_40mV | BRNG_FSR_16V;

    my_INA->calibrationValue = 8192;

    INA219_WriteRegister(my_INA, INA219_REG_CONFIG, config);
    INA219_WriteRegister(my_INA, INA219_REG_CALIBRATION, my_INA->calibrationValue);
}


static void setCalibration_32V_2A(INA219_t *my_INA)
{
    uint16_t config           =   SHUNT_BUS_CONT | SADC_12BIT_532US
                       | BADC_12BIT_532US | PG_GAIN_8_40mV | BRNG_FSR_32V;

    my_INA->calibrationValue = 0x1000;

    INA219_WriteRegister(my_INA, INA219_REG_CONFIG, config);
    INA219_WriteRegister(my_INA, INA219_REG_CALIBRATION, my_INA->calibrationValue);
}


static int16_t INA219_GetVoltage_Raw(INA219_t *my_INA)
{
    my_INA->data_read_buff[0] = INA219_ReadRegister(my_INA, INA219_REG_BUSVOLTAGE);
    return my_INA->data_read_buff[0];
}

static int16_t INA219_GetShuntVoltage_Raw(INA219_t *my_INA)
{
    my_INA->data_read_buff[1] = INA219_ReadRegister(my_INA, INA219_REG_SHUNTVOLTAGE);
    return my_INA->data_read_buff[1];
}


/* NOT DONE */
float INA219_GetVoltage_V(INA219_t *my_INA)
{
    my_INA->voltage_value = INA219_GetVoltage_Raw(my_INA);

    return my_INA->voltage_value;
}

/* NOT DONE */
float INA219_GetShuntVoltage_mV(INA219_t *my_INA)
{
    my_INA->voltage_shunt_value = INA219_GetShuntVoltage_Raw(my_INA);

    return (my_INA->voltage_shunt_value * 0.01);
}

static void INA219_GetCurrent_Raw(INA219_t *my_INA)
{
    if( INA219_ReadRegister(my_INA, INA219_REG_CURRENT) != 1 ) {
        my_INA->data_read_buff[CURRENT] = -1;
        return;
    }
    uint16_t raw = (my_INA->receive[0] << 8) | (my_INA->receive[1]);

    my_INA->data_read_buff[CURRENT] = raw;
}

float INA219_GetCurrent_mA(INA219_t *my_INA)
{
    int16_t const CURRENT_DIVIDER_MA = 20;
    int16_t const CURRENT_MULT_mW = 1.0f;
    INA219_GetCurrent_Raw(my_INA);
    my_INA->current_value = my_INA->data_read_buff[CURRENT];

    if ( my_INA->current_value == -1 ) {
        return -1;
    }

    return my_INA->current_value / CURRENT_DIVIDER_MA;
}

//int INA219_init(INA219_t *my_INA, I2C_HandleTypeDef *I2C_handle)
int INA219_init(INA219_t *my_INA)
{
//    my_INA->I2C_handle          = I2C_handle;
    my_INA->calibrationValue    = 0;
    my_INA->data_read_buff[3]   = 0;
    my_INA->voltage_value       = 0;
    my_INA->voltage_shunt_value = 0;
    my_INA->current_value       = 0;

    setCalibration_16V_400mA(my_INA);

    return 1;
}
