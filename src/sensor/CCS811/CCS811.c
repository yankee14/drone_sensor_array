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

/**
 * Ensure the device is out of BOOT mode, and into running mode
 */
void initCCS811(uint32_t drive_mode)
{
    uint32_t status = get_STATUS();

    if( !(status & CCS811_STATUS_FW_MODE_MASK) ) { // if we are in BOOT mode
        I2C0start();
        I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
        I2C0write(CCS811_APP_START_ADDR);
        I2C0stop();
    }

    status = get_STATUS();
    if( !(status & CCS811_STATUS_FW_MODE_MASK) ) // check again
        for(;;); // don't continue

    set_MEAS_MODE(drive_mode << CCS811_DRIVE_MODE);
}

uint32_t get_STATUS(void)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_STATUS_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);
    uint32_t status = I2C0read(I2CNACK);
    I2C0stop();
    return status & CCS811_STATUS_RO_MASK;
}

uint32_t get_MEAS_MODE(void)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_MEAS_MODE_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);
    uint32_t meas_mode = I2C0read(I2CNACK);
    I2C0stop();
    return meas_mode & CCS811_MEAS_MODE_RW_MASK;
}

uint32_t* get_ALG_RESULT_DATA(uint32_t* data, uint32_t numBytes)
{
    if(numBytes > CCS811_ALG_RESULT_DATA_NUM_BYTES)
        numBytes = CCS811_ALG_RESULT_DATA_NUM_BYTES;
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_ALG_RESULT_DATA_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);

    for(uint32_t i = 0; i < numBytes - 1; i++)
        data[i] = I2C0read(I2CACK);

    data[numBytes - 1] = I2C0read(I2CNACK); // end with NACK

    I2C0stop();
    return data;
}

uint32_t* get_RAW_DATA(uint32_t* data)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_RAW_DATA_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);

    for(uint32_t i = 0; i < CCS811_RAW_DATA_NUM_BYTES - 1; i++)
        data[i] = I2C0read(I2CACK);

    data[CCS811_RAW_DATA_NUM_BYTES - 1] = I2C0read(I2CNACK); // end with NACK

    I2C0stop();
    return data;
}

uint32_t* get_NTC(uint32_t* data)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_NTC_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);

    for(uint32_t i = 0; i < CCS811_NTC_NUM_BYTES - 1; i++)
        data[i] = I2C0read(I2CACK);

    data[CCS811_NTC_NUM_BYTES - 1] = I2C0read(I2CNACK); // end with NACK

    I2C0stop();
    return data;
}

uint32_t get_HW_ID(void)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_HW_ID_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);
    uint32_t hw_id = I2C0read(I2CNACK);
    I2C0stop();
    return hw_id;
}

uint32_t get_ERROR_ID(void)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_ERROR_ID_ADDR);
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CREAD);
    uint32_t error_id = I2C0read(I2CNACK);
    I2C0stop();
    return error_id;
}

void set_MEAS_MODE(uint32_t data)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR << 1) | I2CWRITE);
    I2C0write(CCS811_MEAS_MODE_ADDR);
    I2C0write(data);
    I2C0stop();
}

void set_ENV_DATA(uint32_t* data)
{
    I2C0start();
    I2C0write( (CCS811_I2C_ADDR) | I2CWRITE);
    I2C0write(CCS811_ENV_DATA_ADDR);

    for(uint32_t i = 0; i < CCS811_ENV_DATA_NUM_BYTES; i++)
        I2C0write(data[i]);

    I2C0stop();
}
