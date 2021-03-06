/*
===============================================================================
 Name        : drone_sensor_array.c
 Author      : Ed Peguillan III
 Version     : R0.1a
 Description : Test bed for UART GPS sesnor and IIC environmental sensors for
               drone project @ Univ of Oklahoma
===============================================================================
*/

#include <cr_section_macros.h>
#include "util/delay.h"
#include "util/i2c.h"
#include <time.h>
#include "sensor/BME280_driver/bme280.h" // hocus pocus pressure sensor math
#include "sensor/CCS811/CCS811.h" // CO2 and TVOC sensor

#ifndef USER_I2C_DEBUG
//#define USER_I2C_DEBUG
#endif

void init(void);
void initCCS811(uint32_t nINTonDataThreshold, uint32_t nINTonDataReady, uint32_t drive_mode);

void testCCS811(void);

void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
void print_bme280_data(struct bme280_data *comp_data);
void handle_bme280_error(int8_t _rslt);

void init(void)
{
    I2C0init2(5, 5); // setup I2C bus, freq 100 kHz

    initCCS811(CCS811_INT_THRESH_DISABLE, CCS811_INT_DATARDY_ENABLE, CCS811_DRIVE_MODE_1); // set 1 Hz readings
}

/**
 * Ensure the device is out of BOOT mode, and into running mode
 */
void initCCS811(uint32_t nINTonDataThreshold, uint32_t nINTonDataReady, uint32_t drive_mode)
{
    uint32_t status = get_STATUS();

    if( !(status & CCS811_STATUS_FW_MODE_MASK) ) {  // if we are in BOOT mode
        uint32_t reg_addr = CCS811_APP_START_ADDR;  // get out of BOOT mode
        I2C0write(CCS811_I2C_ADDR_SEC, &reg_addr, 1, 1);
    }

    set_MEAS_MODE(
            (nINTonDataThreshold << CCS811_INT_THRESH) |
            (nINTonDataReady << CCS811_INT_DATARDY) |
            (drive_mode << CCS811_DRIVE_MODE));
}

void testCCS811(void)
{

//    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register
//
//    printf("Status: 0x%02X\n", get_STATUS()); // print status register
//
//    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register
//
//    printf("Hardware ID read: 0x%02X, should read: 0x81\n", get_HW_ID()); // print HW_ID register
//
//    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    uint32_t result[8];
    get_ALG_RESULT_DATA(result, 8); // get CO2 and TVOC

//    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register
//
    printf("Result: ");
    for(uint32_t i = 0; i < 8; i++)
        printf("0x%02X ", result[i]); // print CO2, TVOC, status register, error register, and RAW_DATA
    printf("\n");
//
//    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register
//
//    printf("Status: 0x%02X\n", get_STATUS()); // print status register
//
//    printf("Error: 0x%02X\n", get_ERROR_ID()); // print error register
//
//    printf("Measure Mode: 0x%02X\n\n", get_MEAS_MODE()); // print measure_mode register
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    _delay_ms(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure *//** Insert this after the read code */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    uint32_t reg_addr_32 = reg_addr & 0xFF;
    rslt = I2C0write(BME280_I2C_ADDR_SEC, &reg_addr_32, 1, 1);
    if(!rslt)
        return -1;

    uint32_t reg_data_32[len];
    for(uint32_t i = 0; i < len; i++)
        reg_data_32[i] = reg_data[i];
    rslt = I2C0read(BME280_I2C_ADDR_SEC, reg_data_32, len, 1);
    if(!rslt)
        return -1;

    for(uint32_t i = 0; i < len; i++)
        reg_data[i] = reg_data_32[i] & 0xFF;

#ifdef USER_I2C_DEBUG
    /** Insert this after the read code */
    printf("RD: %d, %d", reg_addr, len);
    for(uint16_t idx = 0; idx < len; idx++)
        printf(" 0x%x", reg_data[idx]);
    printf("\r\n");
#endif

    return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

#ifdef USER_I2C_DEBUG
    /** Insert this before the write code */
    printf("WR: %d, %d", reg_addr, len);
    for(uint16_t idx = 0; idx < len; idx++)
        printf(" 0x%x", reg_data[idx]);
    printf("\r\n");
#endif

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          | NOTE NO RESTART CONDITION ALLOWED
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    /*
     * BAD CODE, HAS I2C RESTART AFTER REG WRITE
    uint32_t reg_addr_32 = reg_addr & 0xFF;

    rslt = I2C0write(BME280_I2C_ADDR_SEC, &reg_addr_32, 1, 0);
    if(!rslt)
        return -1;

    uint32_t reg_data_32[len];
    for(uint32_t i = 0; i < len; i++)
        reg_data_32[i] = reg_data[i] & 0xFF;

    rslt = I2C0write(BME280_I2C_ADDR_SEC, reg_data_32, len, 1);
    if(!rslt)
        return -1;
     */

    uint32_t reg_32[len + 1]; // hold reg_addr, followed by all data
    reg_32[0] = reg_addr & 0xFF;

    for(uint32_t i = 1; i < len + 1; i++)
        reg_32[i] = reg_data[i - 1] & 0xFF;

    rslt = I2C0write(dev_id, reg_32, len + 1, 1);
    if(!rslt)
        return -1;

    return 0;
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

    /* Delay while the sensor completes a measurement */
    dev->delay_ms(70);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

	return rslt;
}

void print_bme280_data(struct bme280_data *comp_data)
{
    printf("           BME280 Data:\n");
	printf("    Temperature    |    Pressure    |    Humidity\n");
#ifdef BME280_FLOAT_ENABLE
    printf("     %0.2f deg C       %0.2f Pa        %0.2f %%\n\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
    printf("     %ld deg C         %ld Pa          %ld %%\n\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void handle_bme280_error(int8_t _rslt)
{
    if (_rslt < BME280_OK) {
        printf("BME280 Error code: %d", _rslt);
        for(;;); /* Trap if error */
    } else if (_rslt > BME280_OK) {
        printf("BME280 Warning code: %d", _rslt);
    }
}

int main(void)
{
    init();
    struct bme280_dev dev;
    struct bme280_data comp_data;
    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_SEC;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);
    handle_bme280_error(rslt);

    rslt = stream_sensor_data_normal_mode(&dev);
    handle_bme280_error(rslt);

    for(;;) {
        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        printf("UTC time and date: %s\n", asctime (timeinfo));

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        handle_bme280_error(rslt);
        print_bme280_data(&comp_data);

        testCCS811();

        uint32_t delay_sec = 5;
        printf("        Delaying %ld seconds...\n\n", delay_sec);
        _delay_ms(1000 * 5); // pause microcontroller
    }

    return 0;
}
