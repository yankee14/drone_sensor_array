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

void init(void);
void initCCS811(uint32_t nINTonDataThreshold, uint32_t nINTonDataReady, uint32_t drive_mode);

void testCCS811(void);

void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
void print_sensor_data(struct bme280_data *comp_data);

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
    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    printf("Current local time and date: %s", asctime (timeinfo));

    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    printf("Status: 0x%02X\n", get_STATUS()); // print status register

    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    printf("Hardware ID read: 0x%02X, should read: 0x81\n", get_HW_ID()); // print HW_ID register

    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    uint32_t result[8];
    get_ALG_RESULT_DATA(result, 8); // get CO2 and TVOC

    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    printf("Result: ");
    for(uint32_t i = 0; i < 8; i++)
        printf("0x%02X ", result[i]); // print CO2, TVOC, status register, error register, and RAW_DATA
    printf("\n");

    printf("Error ID: 0x%02X\n", get_ERROR_ID()); // print error register

    printf("Status: 0x%02X\n", get_STATUS()); // print status register

    printf("Error: 0x%02X\n", get_ERROR_ID()); // print error register

    printf("Measure Mode: 0x%02X\n\n\n", get_MEAS_MODE()); // print measure_mode register
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
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

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
    rslt = I2C0write(BME280_I2C_ADDR_SEC, (uint32_t*)&reg_addr, 1, 1);
    if(!rslt)
        return -1;

    rslt = I2C0read(BME280_I2C_ADDR_SEC, (uint32_t*)reg_data, len, 1);
    if(!rslt)
        return -1;

    return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

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
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    rslt = I2C0write(BME280_I2C_ADDR_SEC, (uint32_t*)&reg_addr, 1, 0);
    if(!rslt)
        return -1;

    rslt = I2C0write(BME280_I2C_ADDR_SEC, (uint32_t*)reg_data, len, 1);
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

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev->delay_ms(70);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

int main(void)
{
    init();

    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_SEC;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);

    for(;;) {
        testCCS811();

        _delay_ms(1000 * 5); // pause microcontroller
    }

    return 0 ;
}
