/*
===============================================================================
 Name        : i2c.c
 Author      : Ed Peguillan III
 Version     : R1.0
 Description : IIC communication utility functions
===============================================================================
*/

#include "i2c.h"

static uint32_t I2C0waitSI(void)
{
    uint32_t timeout = 0;
    while( !(LPC_I2C0->I2CONSET & (1 << SI)) ) {
        if(++timeout > 100000)
            return -1;
    }

    return 0;
}

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

    I2C0stop();
}

inline uint32_t I2C0start(void)
{
    uint32_t status = 0;
    uint32_t isInterrupted = LPC_I2C0->I2CONSET & (1 << SI);

    // 8.1 Before master mode can be entered, I2CON must be initialized to:
    //  - I2EN STA STO SI AA - -
    //  -  1    0   0   x  x - -
    // if AA = 0, it can't enter slave mode
    LPC_I2C0->I2CONCLR = (1 << STAC) | (0 << SIC) | (1 << AAC);

    // The master mode may now be entered by setting the STA bit
    // this will generate a start condition when the bus becomes free
    LPC_I2C0->I2CONSET = (1 << STA) | (1 << AA);
    // Clearing SI bit when it wasn't set on entry can jump past state
    // 0x10 or 0x08 and erroneously send uninitialized slave address.
    if(isInterrupted)
        LPC_I2C0->I2CONCLR = (1 << SIC);

    I2C0waitSI();
    status = LPC_I2C0->I2STAT;

    // Clear start bit now that it's transmitted
    LPC_I2C0->I2CONCLR = (1 << STAC);
    return status;
}

inline uint32_t I2C0stop(void)
{
    uint32_t timeout = 0;

    // write the stop bit
    LPC_I2C0->I2CONSET = (1 << STO);
    LPC_I2C0->I2CONCLR = (1 << SI);

    // wait for STO bit to reset
    while(LPC_I2C0->I2CONSET & (1 << STO)) {
        if (++timeout > 100000)
            return 1;
    }

    return 0;
}

static inline uint32_t I2C0doWrite(uint32_t value, uint32_t address)
{
    // write the data
    LPC_I2C0->I2DAT = value & 0xFF;

    // clear SI to init a send
    LPC_I2C0->I2CONCLR = (1 << SIC);

    // wait and return status
    I2C0waitSI();
    return LPC_I2C0->I2STAT;
}

static inline uint32_t I2C0doRead(uint32_t last)
{
    // we are in state 0x40 (SLA+R tx'd) or 0x50 (data rx'd and ack)
    if(last) {
        LPC_I2C0->I2CONCLR = (1 << AAC);    // send a NOT ACK
    } else {
        LPC_I2C0->I2CONSET = (1 << AA);     // send an ACK
    }

    // accept byte
    LPC_I2C0->I2CONCLR = (1 << SIC);

    // wait for it to arrive
    I2C0waitSI();

    // return the data
    return (LPC_I2C0->I2DAT & 0xFF);
}

// The I2C does a read or a write as a whole operation
// There are two types of error conditions it can encounter
//  1) it can not obtain the bus
//  2) it gets error responses at part of the transmission
//
// We tackle them as follows:
//  1) we retry until we get the bus. we could have a "timeout" if we can not get it
//      which basically turns it in to a 2)
//  2) on error, we use the standard error mechanisms to report/debug
//
// Therefore an I2C transaction should always complete. If it doesn't it is usually
// because something is setup wrong (e.g. wiring), and we don't need to programatically
// check for that

uint32_t I2C0write(uint32_t address, const uint32_t* data, uint32_t length, uint32_t stop)
{
    uint32_t i, status;

    status = I2C0start();
    if ((status != I2STAT_REPEATED_START_XMIT) && (status != I2STAT_SINGLE_START_XMIT)) {
        I2C0stop();
        return I2C_ERROR_BUS_BUSY;
    }

    status = I2C0doWrite( (address << 1) & 0xFE, 1);
    if (status != I2STAT_SLAVE_W_ACK_RECV) {
        I2C0stop();
        return I2C_ERROR_NO_SLAVE;
    }

    for (i=0; i < length; i++) {
        status = I2C0doWrite(data[i], 0);
        if(status != I2STAT_DATA_XMIT_ACK_RECV) {
            I2C0stop();
            return i;
        }
    }

    // clearing the serial interrupt here might cause an unintended rewrite of the last byte
    // see also issue report https://mbed.org/users/mbed_official/code/mbed/issues/1
    // i2c_clear_SI(obj);

    // If not repeated start, send stop.
    if (stop)
        I2C0stop();

    return length;
}

uint32_t I2C0read(uint32_t address, uint32_t* data, uint32_t length, uint32_t stop)
{
    uint32_t count, status;

    status = I2C0start();
    if((status != I2STAT_REPEATED_START_XMIT) && (status != I2STAT_SINGLE_START_XMIT)) {
        I2C0stop();
        return I2C_ERROR_BUS_BUSY;
    }

    status = I2C0doWrite((address << 1) | 0x01, 1);
    if(status != I2STAT_SLAVE_R_ACK_RECV) {
        I2C0stop();
        return I2C_ERROR_NO_SLAVE;
    }

    // Read in all except last byte
    for(count = 0; count < (length - 1); count++) {
        uint32_t value = I2C0doRead(0);
        status = LPC_I2C0->I2STAT;
        if(status != I2STAT_DATA_RECV_ACK_RET) {
            I2C0stop();
            return count;
        }

        data[count] = value & 0xFF;
    }

    // read in last byte
    uint32_t value = I2C0doRead(1);
    status = LPC_I2C0->I2STAT;
    if(status != I2STAT_DATA_RECV_NACK_RET) {
        I2C0stop();
        return length - 1;
    }

    data[count] = value;

    // If not repeated start, send stop.
    if(stop)
        I2C0stop();

    return length;
}
