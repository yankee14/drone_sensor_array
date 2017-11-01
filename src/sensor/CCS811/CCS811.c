/*
===============================================================================
 Name        : CCS811.c
 Author      : Ed Peguillan III
 Version     : R0.1a
 Description : CO2 and VOC sensor register addresses, bit names, and functions
===============================================================================
*/

#include "CCS811.h"

#include "../../util/i2c.h"

uint32_t get_STATUS(void)
{
    uint32_t reg_addr = CCS811_STATUS_ADDR;
    I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 0);

    uint32_t status;
    I2C0read(CCS811_I2C_ADDR_SEC, &status, 1, 1);

    return status & CCS811_STATUS_RO_MASK;
}

uint32_t get_MEAS_MODE(void)
{
    uint32_t status;

    uint32_t reg_addr = CCS811_MEAS_MODE_ADDR;
    status = I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 0);
    if(!status)
        return -1;

    uint32_t meas_mode;
    status = I2C0read(CCS811_I2C_ADDR_SEC, &meas_mode, 1, 1);
    if(!status)
        return -1;

    return meas_mode & CCS811_MEAS_MODE_RW_MASK;
}

uint32_t* get_ALG_RESULT_DATA(uint32_t* data, uint32_t numBytes)
{
    uint32_t status;

    uint32_t reg_addr = CCS811_ALG_RESULT_DATA_ADDR;
    status = I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 0);
    if(!status)
        return 0;

    status = I2C0read(CCS811_I2C_ADDR_SEC, data, numBytes, 1);
    if(!status)
        return 0;

    return data;
}

uint32_t* get_RAW_DATA(uint32_t* data)
{
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CWRITE);
//    I2C0write(CCS811_RAW_DATA_ADDR);
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CREAD);
//
//    for(uint32_t i = 0; i < CCS811_RAW_DATA_NUM_BYTES - 1; i++)
//        data[i] = I2C0read(I2CACK);
//
//    data[CCS811_RAW_DATA_NUM_BYTES - 1] = I2C0read(I2CNACK); // end with NACK
//
//    I2C0stop();
//    return data;

    return 0;
}

uint32_t* get_NTC(uint32_t* data)
{
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CWRITE);
//    I2C0write(CCS811_NTC_ADDR);
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CREAD);
//
//    for(uint32_t i = 0; i < CCS811_NTC_NUM_BYTES - 1; i++)
//        data[i] = I2C0read(I2CACK);
//
//    data[CCS811_NTC_NUM_BYTES - 1] = I2C0read(I2CNACK); // end with NACK
//
//    I2C0stop();
//    return data;

    return 0;
}

uint32_t get_HW_ID(void)
{
    uint32_t status;

    uint32_t reg_addr = CCS811_HW_ID_ADDR;
    status = I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 0);
    if(!status)
        return -1;

    uint32_t hw_id;
    status = I2C0read(CCS811_I2C_ADDR_SEC, &hw_id, 1, 1);
    if(!status)
        return -1;

    return hw_id;
}

uint32_t get_ERROR_ID(void)
{
    uint32_t status;

    uint32_t reg_addr = CCS811_ERROR_ID_ADDR;
    status = I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 0);
    if(!status)
        return -1;

    uint32_t error_id;
    status = I2C0read(CCS811_I2C_ADDR_SEC, &error_id, 1, 1);
    if(!status)
        return -1;

    return error_id;
}

void set_MEAS_MODE(uint32_t data)
{
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CWRITE);
//    I2C0write(CCS811_MEAS_MODE_ADDR);
//    I2C0write(data);
//    I2C0stop();

    uint32_t meas_mode[] = {CCS811_MEAS_MODE_ADDR, data};
    I2C0write(CCS811_I2C_ADDR_SEC, meas_mode, 2, 1);
}

void set_ENV_DATA(uint32_t* data)
{
//    I2C0start();
//    I2C0write( (CCS811_I2C_ADDR_SEC) | I2CWRITE);
//    I2C0write(CCS811_ENV_DATA_ADDR);
//
//    for(uint32_t i = 0; i < CCS811_ENV_DATA_NUM_BYTES; i++)
//        I2C0write(data[i]);
//
//    I2C0stop();
}
