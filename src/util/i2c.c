/*
===============================================================================
 Name        : i2c.c
 Author      : Ed Peguillan III
 Version     : R1.0
 Description : IIC communication utility functions
===============================================================================
*/

#include "i2c.h"

void I2C0init(void)
{
    LPC_PINCON->PINSEL1 |= (1 << 22) | (1 << 24);
    LPC_PINCON->PINSEL1 = (SEL_F1 << P0_27SEL) | (SEL_F1 << P0_28SEL) |
            (LPC_PINCON->PINSEL1 & ~((0b11 << P0_27SEL) | (0b11 << P0_28SEL)));

    LPC_I2C0->I2CONSET = (1 << I2EN);
    LPC_I2C0->I2SCLL = 4;
    LPC_I2C0->I2SCLH = 4;
}

void I2C0init2(uint32_t I2SCLL, uint32_t I2SCLH)
{
    LPC_PINCON->PINSEL1 |= (1 << 22) | (1 << 24);
    LPC_PINCON->PINSEL1 = (SEL_F1 << P0_27SEL) | (SEL_F1 << P0_28SEL) |
            (LPC_PINCON->PINSEL1 & ~((0b11 << P0_27SEL) | (0b11 << P0_28SEL)));

    LPC_I2C0->I2CONSET = (1 << I2EN);
    LPC_I2C0->I2SCLL = I2SCLL & 0xFFFF;
    LPC_I2C0->I2SCLH = I2SCLH & 0xFFFF;
}

void I2C0start(void)
{
    LPC_I2C0->I2CONSET = (1 << SI);             // set my turn
    LPC_I2C0->I2CONSET = (1 << STA);             // prepare start
    LPC_I2C0->I2CONCLR = (1 << SIC);             // set hardware's turn
    while(!(LPC_I2C0->I2CONSET & (1 << SI)));     // wait for start to complete
    LPC_I2C0->I2CONCLR = (1 << STA);            // unprepare start
}

void I2C0stop(void)
{
    LPC_I2C0->I2CONSET = (1 << STO);            // prepare stop (unprepares automatically)
    LPC_I2C0->I2CONCLR = (1 << SIC);            // set hardware's turn
    while(LPC_I2C0->I2CONSET & (1 << SI));      // wait for stop to complete
}

void I2C0write(uint32_t data)
{
    LPC_I2C0->I2DAT = data & I2DAT_RW_MASK;     // prepare data
    LPC_I2C0->I2CONCLR = (1 << SIC);            // set hardware's turn
    while(!(LPC_I2C0->I2CONSET & (1 << SI)));   // wait for transfer complete
    //TODO check I2STAT
}

uint32_t I2C0read(uint32_t ack)
{
    if(ack) {
        LPC_I2C0->I2CONSET = (1 << AA);         // auto acknowledge
    } else {
        LPC_I2C0->I2CONCLR = (1 << AA);         // do not auto-acknowledge
    }

    LPC_I2C0->I2CONCLR = (1 << SI);             // set hardware's turn
    while(!(LPC_I2C0->I2CONSET & (1 << SI)));   // wait for read to complete
    return LPC_I2C0->I2DAT & I2DAT_RW_MASK;
}
