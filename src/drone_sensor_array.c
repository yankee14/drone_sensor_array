/*
===============================================================================
 Name        : drone_sensor_array.c
 Author      : Ed Peguillan III
 Version     : R0.1a
 Description : Test bed for UART GPS sesnor and IIC environmental sensor for
               drone project @ Univ of Oklahoma
===============================================================================
*/

#include "sensor/CCS811.h" // CO2 and TVOC sensor
#include <cr_section_macros.h>
#include "util/delay.h"
#include "util/i2c.h"
#include <stdio.h>

#define ADDR_BME280 0x77

void init(void)
{
    I2C0init2(5, 5); // setup I2C bus, freq 100 kHz

    initCCS811(CCS811_DRIVE_MODE_1); // set 1 Hz readings
}

int main(void)
{
    init();

    for(;;) {
        _delay_ms(2000);

        printf("Error ID: 0x%02X\n", get_ERROR_ID());

        printf("Status: 0x%02X\n", get_STATUS());

        printf("Error ID: 0x%02X\n", get_ERROR_ID());

        printf("Hardware ID read: 0x%02X, should read: 0x81\n", get_HW_ID());

        printf("Error ID: 0x%02X\n", get_ERROR_ID());

        uint32_t result[8];
        get_ALG_RESULT_DATA(result, 8);

        printf("Error ID: 0x%02X\n", get_ERROR_ID());

        printf("Result: ");
        for(uint32_t i = 0; i < 8; i++)
            printf("0x%02X ", result[i]);
        printf("\n");

        printf("Error ID: 0x%02X\n", get_ERROR_ID());

        printf("Status: 0x%02X\n", get_STATUS());

        printf("Error: 0x%02X\n", get_ERROR_ID());

        printf("Measure Mode: 0x%02X\n", get_MEAS_MODE());
    }

    return 0 ;
}
