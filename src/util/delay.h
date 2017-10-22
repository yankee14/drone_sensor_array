/*
===============================================================================
 Name        : i2c.h
 Author      : Ed Peguillan III
 Version     : R0.1b
 Description : Delay CPU by specified number of clock cycles, or by time.
===============================================================================
*/

#ifndef _UTIL_DELAY_H_
#define _UTIL_DELAY_H_

#include "../types.h"

void wait_ticks(const uint32_t count);
uint32_t calc_delay_ms(const uint32_t ms);
void _delay_ms(const uint32_t ms);

#endif
