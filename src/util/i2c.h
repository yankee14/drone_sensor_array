/*
===============================================================================
 Name        : i2c.h
 Author      : Ed Peguillan III
 Version     : R1.0
 Description : IIC communication utility functions
===============================================================================
*/

#ifndef _UTIL_I2C_H_
#define _UTIL_I2C_H_

#include "../LPC1769.h"
#include "../types.h"

#define I2CWRITE 	0b0U
#define I2CREAD		0b1U
#define I2CACK      1
#define I2CNACK     0

void I2C0init(void);
void I2C0init2(uint32_t I2SCLL, uint32_t I2SCLH);
void I2C0start(void);
void I2C0stop(void);
void I2C0write(uint32_t data);
uint32_t I2C0read(uint32_t ack);

#endif
