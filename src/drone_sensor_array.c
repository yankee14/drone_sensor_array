/*
===============================================================================
 Name        : drone_sensor_array.c
 Author      : Ed Peguillan III
 Version     : R0.1a
 Description : Test bed for UART GPS sesnor and IIC environmental sensors for
               drone project @ Univ of Oklahoma
===============================================================================
*/

#include "sensor/CCS811.h" // CO2 and TVOC sensor
#include <cr_section_macros.h>
#include "util/delay.h"
#include "util/i2c.h"
#include <stdio.h>
#include <time.h>

#define ADDR_BME280 0x77

void init(void)
{
    I2C0init2(5, 5); // setup I2C bus, freq 100 kHz

    initCCS811(CCS811_DRIVE_MODE_1); // set 1 Hz readings
}

int main(void)
{
    init();

    time_t rawtime;
    struct tm* timeinfo;

    for(;;) {
        _delay_ms(1000 * 60); // pause microcontroller 2 seconds

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

    return 0 ;
}
