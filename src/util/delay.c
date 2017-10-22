/*
===============================================================================
 Name        : i2c.c
 Author      : Ed Peguillan III
 Version     : R0.1b
 Description : Delay CPU by specified number of clock cycles, or by time.
===============================================================================
*/

#include "delay.h"

#define CLK_ADJ 100 // adjustment factor for clock error

void wait_ticks(const uint32_t count)
{
    volatile int ticks = count;

    while(ticks > 0)
        --ticks;
}

uint32_t calc_delay_ms(const uint32_t ms)
{
	return ms * 400 + CLK_ADJ;
}

void _delay_ms(const uint32_t ms)
{
	wait_ticks(calc_delay_ms(ms));
}
