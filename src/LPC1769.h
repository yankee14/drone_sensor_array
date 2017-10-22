/*
===============================================================================
 Name        : LPC1769.h
 Author      : ARM, Ed Peguillan III
 Version     : R0.5b
 Description : Convenient register and bit names
 Forked from https://raw.githubusercontent.com/ARMmbed/mbed-os/master/targets/TARGET_NXP/TARGET_LPC176X/device/LPC17xx.h
===============================================================================
*/

/**************************************************************************//**
 * @file     LPC17xx.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           NXP LPC17xx Device Series
 * @version: V1.09
 * @date:    17. March 2010
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#ifndef _LPC1769_H_
#define _LPC1769_H_


#include "types.h"

// Generalized port pin positions
#define P0_0  0
#define P0_1  1
#define P0_2  2
#define P0_3  3
#define P0_4  4
#define P0_5  5
#define P0_6  6
#define P0_7  7
#define P0_8  8
#define P0_9  9
#define P0_10 10
#define P0_11 11
//#define P0_12 12
//#define P0_13 13
//#define P0_14 14
#define P0_15 15
#define P0_16 16
#define P0_17 17
#define P0_18 18
//#define P0_19 19
//#define P0_20 20
#define P0_21 21
#define P0_22 22
#define P0_23 23
#define P0_24 24
#define P0_25 25
#define P0_26 26
#define P0_27 27
#define P0_28 28
#define P0_29 29
#define P0_30 30
//#define P0_31 31

// Generalized port pin positions
#define P1_0  0
#define P1_1  1
#define P1_2  2
#define P1_3  3
#define P1_4  4
#define P1_5  5
#define P1_6  6
#define P1_7  7
#define P1_8  8
#define P1_9  9
#define P1_10 10
#define P1_11 11
//#define P1_12 12
//#define P1_13 13
//#define P1_14 14
#define P1_15 15
#define P1_16 16
#define P1_17 17
#define P1_18 18
//#define P1_19 19
//#define P1_20 20
#define P1_21 21
#define P1_22 22
#define P1_23 23
#define P1_24 24
#define P1_25 25
#define P1_26 26
#define P1_27 27
#define P1_28 28
#define P1_29 29
#define P1_30 30
//#define P1_31 31
//
/**
 * Nested Vectored Interrupt Controller (NVIC) ID's
 *
 * Values on pg. 74 and 78-87, LPC176X User Manual.
 */
typedef enum IRQn
{
    TIMER0_IRQn = 1,   // Timer0 Interrupt
    TIMER1_IRQn = 2,   // Timer1 Interrupt
    TIMER2_IRQn = 3,   // Timer2 Interrupt
    TIMER3_IRQn = 4,   // Timer3 Interrupt
    UART0_IRQn  = 5,   // UART0 Interrupt
    UART1_IRQn  = 6,   // UART1 Interrupt
    UART2_IRQn  = 7,   // UART2 Interrupt
    UART3_IRQn  = 8,   // UART3 Interrupt
    PWM1_IRQn   = 9,   // PWM1 Interrupt
    I2C0_IRQn   = 10,  // I2C0 Interrupt
    I2C1_IRQn   = 11,  // I2C1 Interrupt
    I2C2_IRQn   = 12,  // I2C2 Interrupt
    SPI_IRQn    = 13,  // SPI Interrupt

    PLL0_IRQn   = 16,     // PLL0 Lock (Main PLL) Interrupt

    EINT0_IRQn  = 18,   // External Interrupt 0 Interrupt        
    EINT1_IRQn  = 19,   // External Interrupt 1 Interrupt        
    EINT2_IRQn  = 20,   // External Interrupt 2 Interrupt        
    EINT3_IRQn  = 21,   // External Interrupt 3 Interrupt        
    ADC_IRQn    = 22,   // A/D Converter Interrupt               
    
    I2S_IRQn    = 27,   // I2S Interrupt

    RIT_IRQn    = 29,   // Repetitive Interrupt Timer Interrupt

    PLL1_IRQn   = 32    // PLL1 Lock (USB PLL) Interrupt
} IRQn_Type;

/*
 *
 */

/**
 * System Control
 */
typedef struct
{
    vuint32_t FLASHCFG;               // Flash Accelerator Module
         uint32_t RESERVED0[31];
    vuint32_t PLL0CON;                // Clocking and Power Control
    vuint32_t PLL0CFG;
        const vuint32_t PLL0STAT;
    vuint32_t PLL0FEED;
         uint32_t RESERVED1[4];
    vuint32_t PLL1CON;
    vuint32_t PLL1CFG;
        const vuint32_t PLL1STAT;
    vuint32_t PLL1FEED;
         uint32_t RESERVED2[4];
    vuint32_t PCON;
    vuint32_t PCONP;
         uint32_t RESERVED3[15];
    vuint32_t CCLKCFG;
    vuint32_t USBCLKCFG;
    vuint32_t CLKSRCSEL;
    vuint32_t CANSLEEPCLR;
    vuint32_t CANWAKEFLAGS;
         uint32_t RESERVED4[10];
    vuint32_t EXTINT;                 // External Interrupts
         uint32_t RESERVED5;
    vuint32_t EXTMODE;
    vuint32_t EXTPOLAR;
        uint32_t RESERVED6[12];
    vuint32_t RSID;                   // Reset
        uint32_t RESERVED7[7];
    vuint32_t SCS;                    // Syscon Miscellaneous Registers
    vuint32_t IRCTRIM;                // Clock Dividers
    vuint32_t PCLKSEL0;
    vuint32_t PCLKSEL1;
        uint32_t RESERVED8[4];
    vuint32_t USBIntSt;               // USB Device/OTG Interrupt Register
    vuint32_t DMAREQSEL;
    vuint32_t CLKOUTCFG;              // Clock Output Configuration
} LPC_SC_TypeDef;

/* PLL0CON register bits */
#define PLLE0 0
#define PLLC0 1

#define PLLE0_MASK      (1 << PLLE0)
#define PLLC0_MASK      (1 << PLLC0)
#define PLL0CON_RO_MASK 0
#define PLL0CON_RW_MASK (PLLE0_MASK | PLLC0_MASK)
#define PLL0CON_MASK    (PLL0CON_RO_MASK | PLL0CON_RW_MASK)

/* PLL0CFG register bits */
#define MSEL0 0
#define NSEL0 16

#define MSEL0_MASK (0x7FFF << MSEL0);
#define NSEL0_MASK (0xFF << NSEL0);
#define PLL0CFG_RO_MASK 0
#define PLL0CFG_RW_MASK (MSEL0_MASK | NSEL0_MASK)
#define PLL0CFG_MASK (PLL0CFG_RO_MASK | PLL0CFG_RW_MASK)

/* PLL0STAT register bits */
//#define MSEL0 0
//#define NSEL0 16
#define PLLE0_STAT  24
#define PLLC0_STAT  25
#define PLOCK0      26

#define PLLE0_STAT_MASK     (1 << PLLE0_STAT)
#define PLLC0_STAT_MASK     (1 << PLLCO_STAT)
#define PLOCK0_MASK         (1 << PLOCK0)
#define PLL0STAT_RO_MASK    (MSEL0_MASK | NSEL0_MASK | PLLE0_STAT_MASK | PLLC0_STAT_MASK | PLOCK0_MASK)
#define PLL0STAT_MASK (PLL0STAT_RO_MASK)

/* PLL0FEED register bits */
#define PLL0FEED0 0

#define PLL0FEED_WO_MASK (0xFF << PLL0FEED0)
#define PLL0FEED_MASK (PLL0FEED_WO_MASK)

/* PCONP register bits  */
#define PCTIM0      1
#define PCTIM1      2
#define PCUART0     3
#define PCUART1     4
#define PCPWM1      6
#define PCI2C0      7
#define PCSPI       8
#define PCRTC       9
#define PCSSP1      10
#define PCADC       12
#define PCCAN1      13
#define PCCAN2      14
#define PCGPIO      15
#define PCRIT       16
#define PCMCPWM     17
#define PCQEI       18
#define PCI2C1      19
#define PCSSP0      21
#define PCTIM2      22
#define PCTIM3      23
#define PCUART2     24
#define PCUART3     25
#define PCI2C2      26
#define PCI2S       27
#define PCGPDMA     29
#define PCENET      30
#define PCUSB       31

#define PCTIM0_MASK     (1 << PCTIM0)
#define PCTIM1_MASK     (1 << PCTIM1)
#define PCUART0_MASK    (1 << PCUART0)
#define PCUART1_MASK    (1 << PCUART1)
#define PCPWM1_MASK     (1 << PCPWM1)
#define PCI2C0_MASK     (1 << PCI2C0)
#define PCSPI_MASK      (1 << PCSPI)
#define PCRTC_MASK      (1 << PCRTC)
#define PCSSP1_MASK     (1 << PCSSP1)
#define PCADC_MASK      (1 << PCADC)
#define PCCAN1_MASK     (1 << PCCAN1)
#define PCCAN2_MASK     (1 << PCCAN2)
#define PCGPIO_MASK     (1 << PCGPIO)
#define PCRIT_MASK      (1 << PCRIT)
#define PCMCPWM_MASK    (1 << PCMCPWM)
#define PCQEI_MASK      (1 << PCQEI)
#define PCI2C1_MASK     (1 << PCI2C1)
#define PCSSP0_MASK     (1 << PCSSP0)
#define PCTIM2_MASK     (1 << PCTIM2)
#define PCTIM3_MASK     (1 << PCTIM3)
#define PCUART2_MASK    (1 << PCUART2)
#define PCUART3_MASK    (1 << PCUART3)
#define PCI2C2_MASK     (1 << PCI2C2)
#define PCI2S_MASK      (1 << PCI2S)
#define PCGPDMA_MASK    (1 << PCGPDMA)
#define PCENET_MASK     (1 << PCENET)
#define PCUSB_MASK      (1 << PCUSB)
#define PCONP_RO_MASK   0
#define PCONP_WO_MASK   0
#define PCONP_RW_MASK (PCTIM0_MASK | PCTIM1_MASK | PCUART0_MASK | PCUART1_MASK | PCPWM1_MASK | PCI2C0_MASK | PCSPI_MASK | PCRTC_MASK | PCSSP1_MASK | PCADC_MASK | PCCAN1_MASK | PCCAN2_MASK  | PCGPIO_MASK | PCRIT_MASK | PCMCPWM_MASK | PCQEI_MASK | PCI2C1_MASK | PCSSP0_MASK | PCTIM2_MASK | PCTIM3_MASK | PCUART2_MASK | PCUART3_MASK | PCI2C2_MASK | PCI2S_MASK | PCGPDMA_MASK |PCENET_MASK | PCUSB_MASK)

/* CCLKCFG register bits */
#define CCLKSEL 0

#define CCLKSEL_MASK (0xFF << CCLKSEL)
#define CCLKCFG_RO_MASK 0
#define CCLKCFG_RW_MASK (CCLKSEL_MASK)
#define CCLKCFG_MASK (CCLKCFG_RO_MASK | CCLKCFG_RW_MASK)

/* CLKSRCSEL register bits */
#define CLKSRC 0

#define CLKSRC_MASK         (0b11 << CLKSRC)
#define CLKSRCSEL_RO_MASK   0
#define CLKSRCSEL_RW_MASK   (CLKSRC_MASK)
#define CLKSRCSEL_MASK      (CLKSRCSEL_RO_MASK | CLKSRCSEL_RW_MASK)

// CLKSRCSEL values
#define CLKSRC_IRC      0b00
#define CLKSRC_MAIN     0b01
#define CLKSRC_RTC      0b10

/* PCLKSEL0 register bits */
#define PCLK_WDT    0
#define PCLK_TIMER0 2
#define PCLK_TIMER1 4
#define PCLK_UART0  6
#define PCLK_UART1  8
#define PCLK_PWM1   12
#define PCLK_I2C0   14
#define PCLK_SPI    16
#define PCLK_SSP1   20
#define PCLK_DAC    22
#define PCLK_ADC    24
#define PCLK_CAN1   26
#define PCLK_CAN2   28
#define PCLK_ACF    30

/* PCLKSEL1 register bits */
#define PCLK_QEI        0
#define PCLK_GPIOINT    2
#define PCLK_PCB        4
#define PCLK_I2C1       6
#define PCLK_SSP0       10
#define PCLK_TIMER2     12
#define PCLK_TIMER3     14
#define PCLK_UART2      16
#define PCLK_UART3      18
#define PCLK_I2C2       20
#define PCLK_I2S        22
#define PCLK_RIT        26
#define PCLK_SYSCON     28
#define PCLK_MC         30

// CLKSEL0/1 values
#define CCLKDIV4    0b00
#define CCLKDIV1    0b01
#define CCLKDIV2    0b10
#define CCLKDIV8    0b11

// EXTINT register bits
#define EINT0 0
#define EINT1 1
#define EINT2 2
#define EINT3 3

// EXTMODE register bits
#define EXTMODE0 0
#define EXTMODE1 1
#define EXTMODE2 2
#define EXTMODE3 3

// EXTPOLAR register bits
#define EXTPOLAR0 0
#define EXTPOLAR1 1
#define EXTPOLAR2 2
#define EXTPOLAR3 3

/* SCS register bits */
#define OSCRANGE    4
#define OSCEN       5
#define OSCSTAT     6

#define OSCRANGE_MASK   (1 << OSCRANGE)
#define OSCEN_MASK      (1 << OSCEN)
#define OSCSTAT_MASK    (1 << OSCSTAT)
#define SCS_RO_MASK     (OSCSTAT_MASK)
#define SCS_RW_MASK     (OSCRANGE_MASK | OSCEN_MASK)
#define SCS_MASK        (SCS_RO_MASK | SCS_RW_MASK)

/* CLKOUTCFG register bits */
#define CLKOUTSEL   0
#define CLKOUTDIV   4
#define CLKOUT_EN   8
#define CLKOUT_ACT  9

#define CLKOUTSEL_MASK  (0b1111 << CLKOUTSEL)
#define CLKOUTDIV_MASK  (0b1111 << CLKOUTDIV)
#define CLKOUT_EN_MASK  (0b1 << CLKOUT_EN)
#define CLKOUT_ACT_MASK (0b1 << CLKOUT_ACT)
#define CLKOUTCFG_MASK  (CLKOUTSEL_MASK | CLKOUTDIV_MASK | CLKOUT_EN_MASK | CLKOUT_ACT_MASK)

// CLKOUTCFG values
#define CLKOUTSEL_CPU   0b0000
#define CLKOUTSEL_MAIN  0b0001
#define CLKOUTSEL_IRC   0b0010
#define CLKOUTSEL_USB   0b0011
#define CLKOUTSEL_RTC   0b0100

/* Nested Vectored Interrupt Control (NVIC) */
/**
 * 
 *
 * Values, offsets, and names on pg. 13, 77 LPC176X User Manual.
 */
typedef struct
{
  vuint32_t ISER[8U]; // Interrupt Set Enable Register
        uint32_t RESERVED0[24U];
  vuint32_t ICER[8U]; // Interrupt Clear Enable Register
        uint32_t RSERVED1[24U];
  vuint32_t ISPR[8U]; // Interrupt Set Pending Register
        uint32_t RESERVED2[24U];
  vuint32_t ICPR[8U]; // Interrupt Clear Pending Register
        uint32_t RESERVED3[24U];
  vuint32_t IABR[8U]; // Interrupt Active bit Register
        uint32_t RESERVED4[56U];
  vuint8_t  IP[240U]; // Interrupt Priority Register (8Bit wide)
        uint32_t RESERVED5[644U];
  vuint32_t STIR;     // Software Trigger Interrupt Register
}  LPC_NVIC_TypeDef;

// ISER0 (RW) register bits
#define ISE_EINT0   18
#define ISE_EINT1   19
#define ISE_EINT2   20
#define ISE_EINT3   21
#define ISE_ADC     22

#define ISE_RIT     29

// ICER0 (RW) register bits
#define ICE_EINT0   18
#define ICE_EINT1   19
#define ICE_EINT2   20
#define ICE_EINT3   21
#define ICE_ADC     22

#define ICE_RIT     29

// ISPR0 (RW) register bits
#define ISP_EINT0   18
#define ISP_EINT1   19
#define ISP_EINT2   20
#define ISP_EINT3   21
#define ISP_ADC     22

#define ISP_RIT     29

// ICPR0 (RW) register bits
#define ICP_EINT0   18
#define ICP_EINT1   19
#define ICP_EINT2   20
#define ICP_EINT3   21
#define ICP_ADC     22

#define ICP_RIT     29

// IABR0 (RW) register bits
#define IAB_EINT0   18
#define IAB_EINT1   19
#define IAB_EINT2   20
#define IAB_EINT3   21
#define IAB_ADC     22

#define IAB_RIT     29

/* Pin Connect (PINCON) */
/**
 * A structure of similar 32-bit widths, enables abstracted
 * PINCON register naming and addressing.
 *
 * Register selects alternate functions and pullup/pulldown/OD.
 *
 * Values, offsets, and names on pg. 116, LPC176X User Manual.
 */
typedef struct
{
    vuint32_t PINSEL0;
    vuint32_t PINSEL1;
    vuint32_t PINSEL2;
    vuint32_t PINSEL3;
    vuint32_t PINSEL4;
    vuint32_t PINSEL5;
    vuint32_t PINSEL6;
    vuint32_t PINSEL7;
    vuint32_t PINSEL8;
    vuint32_t PINSEL9;
    vuint32_t PINSEL10;
        uint32_t RESERVED0[5];
    vuint32_t PINMODE0;
    vuint32_t PINMODE1;
    vuint32_t PINMODE2;
    vuint32_t PINMODE3;
    vuint32_t PINMODE4;
    vuint32_t PINMODE5;
    vuint32_t PINMODE6;
    vuint32_t PINMODE7;
    vuint32_t PINMODE8;
    vuint32_t PINMODE9;
    vuint32_t PINMODE_OD0;
    vuint32_t PINMODE_OD1;
    vuint32_t PINMODE_OD2;
    vuint32_t PINMODE_OD3;
    vuint32_t PINMODE_OD4;
    vuint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

#define SEL_GPIO    0b00
#define SEL_F1      0b01
#define SEL_F2      0b10
#define SEL_F3      0b11

#define MODE_PU     0b00
#define MODE_REPEAT 0b01
#define MODE_OD     0b10
#define MODE_PD     0b11

#define ODMODE_NORM 0b0
#define ODMODE_OD   0b1

// PINSEL0 (RW) register bits
#define P0_0SEL  0
#define P0_1SEL  2
#define P0_2SEL  4
#define P0_3SEL  6
#define P0_4SEL  8
#define P0_5SEL  10
#define P0_6SEL  12
#define P0_7SEL  14
#define P0_8SEL  16
#define P0_9SEL  18
#define P0_10SEL 20
#define P0_11SEL 22
//#define P0_12SEL 24
//#define P0_13SEL 26
//#define P0_14SEL 28
#define P0_15SEL 30

// PINSEL1 (RW) register bits
#define P0_16SEL 0
#define P0_17SEL 2
#define P0_18SEL 4
//#define P0_19SEL 6
//#define P0_20SEL 8
#define P0_21SEL 10
#define P0_22SEL 12
#define P0_23SEL 14
#define P0_24SEL 16
#define P0_25SEL 18
#define P0_26SEL 20
#define P0_27SEL 22
#define P0_28SEL 24
#define P0_29SEL 26
#define P0_30SEL 28
//#define P0_31SEL 30

// PINSEL3 (RW) register bits
#define P1_16SEL 0
#define P1_17SEL 2
#define P1_18SEL 4
//#define P1_19SEL 6
//#define P1_20SEL 8
#define P1_21SEL 10
#define P1_22SEL 12
#define P1_23SEL 14
#define P1_24SEL 16
#define P1_25SEL 18
#define P1_26SEL 20
#define P1_27SEL 22
#define P1_28SEL 24
#define P1_29SEL 26
#define P1_30SEL 28
//#define P1_31SEL 30

// PINMODE0 (RW) register bits
#define P0_0MODE  0
#define P0_1MODE  2
#define P0_2MODE  4
#define P0_3MODE  6
#define P0_4MODE  8
#define P0_5MODE  10
#define P0_6MODE  12
#define P0_7MODE  14
#define P0_8MODE  16
#define P0_9MODE  18
#define P0_10MODE 20
#define P0_11MODE 22
//#define P0_12MODE 24
//#define P0_13MODE 26
//#define P0_14MODE 28
#define P0_15MODE 30

// PINMODE1 (RW) register bits
#define P0_16MODE 0
#define P0_17MODE 2
#define P0_18MODE 4
//#define P0_19MODE 6
//#define P0_20MODE 8
#define P0_21MODE 10
#define P0_22MODE 12
#define P0_23MODE 14
#define P0_24MODE 16
#define P0_25MODE 18
#define P0_26MODE 20
#define P0_27MODE 22
#define P0_28MODE 24
#define P0_29MODE 26
#define P0_30MODE 28
//#define P0_31MODE 30

// PINMODE3 (RW) register bits
#define P1_16MODE 0
#define P1_17MODE 2
#define P1_18MODE 4
//#define P1_19MODE 6
//#define P1_20MODE 8
#define P1_21MODE 10
#define P1_22MODE 12
#define P1_23MODE 14
#define P1_24MODE 16
#define P1_25MODE 18
#define P1_26MODE 20
#define P1_27MODE 22
#define P1_28MODE 24
#define P1_29MODE 26
#define P1_30MODE 28
//#define P1_31MODE 30

// PINMODE_OD0 register bits
#define P0_0OD  0
#define P0_1OD  1
#define P0_2OD  2
#define P0_3OD  3
#define P0_4OD  4
#define P0_5OD  5
#define P0_6OD  6
#define P0_7OD  7
#define P0_8OD  8
#define P0_9OD  9
#define P0_10OD 10
#define P0_11OD 11
//#define P0_12OD 12
//#define P0_13OD 13
//#define P0_14OD 14
#define P0_15OD 15
#define P0_16OD 16
#define P0_17OD 17
#define P0_18OD 18
//#define P0_19OD 19
//#define P0_20OD 20
#define P0_21OD 21
#define P0_22OD 22
#define P0_23OD 23
#define P0_24OD 24
#define P0_25OD 25
#define P0_26OD 26
#define P0_27OD 27
#define P0_28OD 28
#define P0_29OD 29
#define P0_30OD 30
//#define P0_31OD 31

/* General Purpose Input/Output (GPIO) */
/**
 * A structure of anonymous unions of similar 32-bit widths.
 * Enables abstracted GPIO register naming and addressing.
 *
 * Values, offsets, and names on pg. 131, LPC176X User Manual.
 */
typedef struct
{
    union {
        vuint32_t FIODIR; // you can talk to all 32 bits at once
        struct {
            vuint16_t FIODIRL; // or two 16-bit chunks
            vuint16_t FIODIRH;
        };
        struct {
            vuint8_t  FIODIR0; // or four 4-bit chunks
            vuint8_t  FIODIR1;
            vuint8_t  FIODIR2;
            vuint8_t  FIODIR3;
        };
    };
        uint32_t RESERVED0[3];
    union {
        vuint32_t FIOMASK;
        struct {
            vuint16_t FIOMASKL;
            vuint16_t FIOMASKH;
        };
        struct {
            vuint8_t  FIOMASK0;
            vuint8_t  FIOMASK1;
            vuint8_t  FIOMASK2;
            vuint8_t  FIOMASK3;
        };
    };
    union {
        vuint32_t FIOPIN;
        struct {
            vuint16_t FIOPINL;
            vuint16_t FIOPINH;
        };
        struct {
            vuint8_t  FIOPIN0;
            vuint8_t  FIOPIN1;
            vuint8_t  FIOPIN2;
            vuint8_t  FIOPIN3;
        };
    };
    union {
        vuint32_t FIOSET;
        struct {
            vuint16_t FIOSETL;
            vuint16_t FIOSETH;
        };
        struct {
            vuint8_t  FIOSET0;
            vuint8_t  FIOSET1;
            vuint8_t  FIOSET2;
            vuint8_t  FIOSET3;
        };
    };
    union {
        vuint32_t FIOCLR;
        struct {
            vuint16_t FIOCLRL;
            vuint16_t FIOCLRH;
        };
        struct {
            vuint8_t  FIOCLR0;
            vuint8_t  FIOCLR1;
            vuint8_t  FIOCLR2;
            vuint8_t  FIOCLR3;
        };
    };
} LPC_GPIO_TypeDef;

#define DIR_INPUT 0
#define DIR_OUTPUT 1

#define GPIO_HI 1
#define GPIO_LO 0

// Generalized port pin values can be used for bit positions

/* GPIO Inerrupts (GPIOINT) */
/**
 * A structure of anonymous unions of similar 32-bit widths.
 * Enables abstracted GPIO interrupt register naming and
 * addressing.
 *
 * Values, offsets, and names on pg. 132, LPC176X User Manual.
 */
typedef struct
{
    const uint32_t IntStatus; // const registers are read-only
    const uint32_t IO0IntStatR;
    const uint32_t IO0IntStatF;
    vuint32_t IO0IntClr;
    vuint32_t IO0IntEnR;
    vuint32_t IO0IntEnF;
        uint32_t RESERVED0[3];
    const uint32_t IO2IntStatR;
    const uint32_t IO2IntStatF;
    vuint32_t IO2IntClr;
    vuint32_t IO2IntEnR;
    vuint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;

// IIOIntStatus (RO) register bits
#define P0Int 0
#define P2Int 2

// IO0IntEnR (RW) register bits
#define P0_0ER  0
#define P0_1ER  1
#define P0_2ER  2
#define P0_3ER  3
#define P0_4ER  4
#define P0_5ER  5
#define P0_6ER  6
#define P0_7ER  7
#define P0_8ER  8
#define P0_9ER  9
#define P0_10ER 10
#define P0_11ER 11
//#define P0_12ER 12
//#define P0_13ER 13
//#define P0_14ER 14
#define P0_15ER 15
#define P0_16ER 16
#define P0_17ER 17
#define P0_18ER 18
//#define P0_19ER 19
//#define P0_20ER 20
#define P0_21ER 21
#define P0_22ER 22
#define P0_23ER 23
#define P0_24ER 24
#define P0_25ER 25
#define P0_26ER 26
#define P0_27ER 27
#define P0_28ER 28
#define P0_29ER 29
#define P0_30ER 30
//#define P0_31ER 31

// IO0IntEnF (RW) register bits
#define P0_0EF  0
#define P0_1EF  1
#define P0_2EF  2
#define P0_3EF  3
#define P0_4EF  4
#define P0_5EF  5
#define P0_6EF  6
#define P0_7EF  7
#define P0_8EF  8
#define P0_9EF  9
#define P0_10EF 10
#define P0_11EF 11
//#define P0_12EF 12
//#define P0_13EF 13
//#define P0_14EF 14
#define P0_15EF 15
#define P0_16EF 16
#define P0_17EF 17
#define P0_18EF 18
//#define P0_19EF 19
//#define P0_20EF 20
#define P0_21EF 21
#define P0_22EF 22
#define P0_23EF 23
#define P0_24EF 24
#define P0_25EF 25
#define P0_26EF 26
#define P0_27EF 27
#define P0_28EF 28
#define P0_29EF 29
#define P0_30EF 30
//#define P0_31EF 31

// IO0IntStatR (RO) register bits
#define P0_0REI  0
#define P0_1REI  1
#define P0_2REI  2
#define P0_3REI  3
#define P0_4REI  4
#define P0_5REI  5
#define P0_6REI  6
#define P0_7REI  7
#define P0_8REI  8
#define P0_9REI  9
#define P0_10REI 10
#define P0_11REI 11
//#define P0_12REI 12
//#define P0_13REI 13
//#define P0_14REI 14
#define P0_15REI 15
#define P0_16REI 16
#define P0_17REI 17
#define P0_18REI 18
//#define P0_19REI 19
//#define P0_20REI 20
#define P0_21REI 21
#define P0_22REI 22
#define P0_23REI 23
#define P0_24REI 24
#define P0_25REI 25
#define P0_26REI 26
#define P0_27REI 27
#define P0_28REI 28
#define P0_29REI 29
#define P0_30REI 30
//#define P0_31REI 31

// IO0IntStatF (RO) register bits
#define P0_0FEI  0
#define P0_1FEI  1
#define P0_2FEI  2
#define P0_3FEI  3
#define P0_4FEI  4
#define P0_5FEI  5
#define P0_6FEI  6
#define P0_7FEI  7
#define P0_8FEI  8
#define P0_9FEI  9
#define P0_10FEI 10
#define P0_11FEI 11
//#define P0_12FEI 12
//#define P0_13FEI 13
//#define P0_14FEI 14
#define P0_15FEI 15
#define P0_16FEI 16
#define P0_17FEI 17
#define P0_18FEI 18
//#define P0_19FEI 19
//#define P0_20FEI 20
#define P0_21FEI 21
#define P0_22FEI 22
#define P0_23FEI 23
#define P0_24FEI 24
#define P0_25FEI 25
#define P0_26FEI 26
#define P0_27FEI 27
#define P0_28FEI 28
#define P0_29FEI 29
#define P0_30FEI 30
//#define P0_31FEI 31

// IO0IntClr (WO) register bits
#define P0_0CI  0
#define P0_1CI  1
#define P0_2CI  2
#define P0_3CI  3
#define P0_4CI  4
#define P0_5CI  5
#define P0_6CI  6
#define P0_7CI  7
#define P0_8CI  8
#define P0_9CI  9
#define P0_10CI 10
#define P0_11CI 11
//#define P0_12CI 12
//#define P0_13CI 13
//#define P0_14CI 14
#define P0_15CI 15
#define P0_16CI 16
#define P0_17CI 17
#define P0_18CI 18
//#define P0_19CI 19
//#define P0_20CI 20
#define P0_21CI 21
#define P0_22CI 22
#define P0_23CI 23
#define P0_24CI 24
#define P0_25CI 25
#define P0_26CI 26
#define P0_27CI 27
#define P0_28CI 28
#define P0_29CI 29
#define P0_30CI 30
//#define P0_31CI 31

/*------------- Timer (TIM) --------------------------------------------------*/
typedef struct
{
    vuint32_t IR;
    vuint32_t TCR;
    vuint32_t TC;
    vuint32_t PR;
    vuint32_t PC;
    vuint32_t MCR;
    vuint32_t MR0;
    vuint32_t MR1;
    vuint32_t MR2;
    vuint32_t MR3;
    vuint32_t CCR;
    const vuint32_t CR0;
    const vuint32_t CR1;
        uint32_t RESERVED0[2];
    vuint32_t EMR;
        uint32_t RESERVED1[12];
    vuint32_t CTCR;
} LPC_TIM_TypeDef;

//TODO setup register bits

/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
typedef struct
{
    vuint32_t IR;
    vuint32_t TCR;
    vuint32_t TC;
    vuint32_t PR;
    vuint32_t PC;
    vuint32_t MCR;
    vuint32_t MR0;
    vuint32_t MR1;
    vuint32_t MR2;
    vuint32_t MR3;
    vuint32_t CCR;
    const vuint32_t CR0;
    const vuint32_t CR1;
    const vuint32_t CR2;
    const vuint32_t CR3;
        uint32_t RESERVED0;
    vuint32_t MR4;
    vuint32_t MR5;
    vuint32_t MR6;
    vuint32_t PCR;
    vuint32_t LER;
        uint32_t RESERVED1[7];
    vuint32_t CTCR;
} LPC_PWM_TypeDef;

//TODO setup register bits

/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
typedef struct
{
    union {
        const vuint8_t  RBR;
        vuint8_t  THR;
        vuint8_t  DLL;
            uint32_t RESERVED0;
    };
    union {
        vuint8_t  DLM;
        vuint32_t IER;
    };
    union {
        const vuint32_t IIR;
        vuint8_t  FCR;
    };
    vuint8_t  LCR;
        uint8_t  RESERVED1[7];
    const vuint8_t  LSR;
        uint8_t  RESERVED2[7];
    vuint8_t  SCR;
        uint8_t  RESERVED3[3];
    vuint32_t ACR;
    vuint8_t  ICR;
        uint8_t  RESERVED4[3];
    vuint8_t  FDR;
        uint8_t  RESERVED5[7];
    vuint8_t  TER;
        uint8_t  RESERVED6[39];
    vuint32_t FIFOLVL;
} LPC_UART_TypeDef;

//TODO setup register bits

typedef struct
{
    union {
        const vuint8_t  RBR;
        vuint8_t  THR;
        vuint8_t  DLL;
            uint32_t RESERVED0;
    };
    union {
        vuint8_t  DLM;
        vuint32_t IER;
    };
    union {
        const vuint32_t IIR;
        vuint8_t  FCR;
    };
    vuint8_t  LCR;
        uint8_t  RESERVED1[7];
    const vuint8_t  LSR;
        uint8_t  RESERVED2[7];
    vuint8_t  SCR;
        uint8_t  RESERVED3[3];
    vuint32_t ACR;
    vuint8_t  ICR;
        uint8_t  RESERVED4[3];
    vuint8_t  FDR;
        uint8_t  RESERVED5[7];
    vuint8_t  TER;
        uint8_t  RESERVED6[39];
    vuint32_t FIFOLVL;
} LPC_UART0_TypeDef;

//TODO setup register bits

typedef struct
{
    union {
        const vuint8_t  RBR;
        vuint8_t  THR;
        vuint8_t  DLL;
            uint32_t RESERVED0;
    };
    union {
        vuint8_t  DLM;
        vuint32_t IER;
    };
    union {
        const vuint32_t IIR;
        vuint8_t  FCR;
    };
    vuint8_t  LCR;
        uint8_t  RESERVED1[3];
    vuint8_t  MCR;
        uint8_t  RESERVED2[3];
    const vuint8_t  LSR;
        uint8_t  RESERVED3[3];
    const vuint8_t  MSR;
        uint8_t  RESERVED4[3];
    vuint8_t  SCR;
        uint8_t  RESERVED5[3];
    vuint32_t ACR;
        uint32_t RESERVED6;
    vuint32_t FDR;
        uint32_t RESERVED7;
    vuint8_t  TER;
        uint8_t  RESERVED8[27];
    vuint8_t  RS485CTRL;
        uint8_t  RESERVED9[3];
    vuint8_t  ADRMATCH;
        uint8_t  RESERVED10[3];
    vuint8_t  RS485DLY;
        uint8_t  RESERVED11[3];
    vuint32_t FIFOLVL;
} LPC_UART1_TypeDef;

//TODO setup register bits

/*------------- Inter-Integrated Circuit (I2C) -------------------------------*/
typedef struct
{
    vuint32_t I2CONSET;
    const vuint32_t I2STAT;
    vuint32_t I2DAT;
    vuint32_t I2ADR0;
    vuint32_t I2SCLH;
    vuint32_t I2SCLL;
    vuint32_t I2CONCLR;
    vuint32_t MMCTRL;
    vuint32_t I2ADR1;
    vuint32_t I2ADR2;
    vuint32_t I2ADR3;
    const vuint32_t I2DATA_BUFFER;
    vuint32_t I2MASK0;
    vuint32_t I2MASK1;
    vuint32_t I2MASK2;
    vuint32_t I2MASK3;
} LPC_I2C_TypeDef;

/* I2CONSET register bits */
#define AA      2
#define SI      3
#define STO     4
#define STA     5
#define I2EN    6

#define AA_MASK     (1 << AA)
#define SI_MASK     (1 << SI)
#define STO_MASK    (1 << STO)
#define STA_MASK    (1 << STA)
#define I2EN_MASK   (1 << I2EN)
#define I2CONSET_RO_MASK 0
#define I2CONSET_WO_MASK 0
#define I2CONSET_RW_MASK (AA_MASK | SI_MASK | STO_MASK | STA_MASK | I2EN_MASK)

/* I2STAT register bits */
#define Status  3//:7

#define Status_MASK (1 << Status)
#define I2STAT_RO_MASK Status_MASK
#define I2STAT_WO_MASK 0
#define I2STAT_RW_MASK 0

/* I2DAT register bits */
#define Data    0//:7

#define Data_MASK (0xFF << Data)
#define I2DAT_RO_MASK 0
#define I2DAT_WO_MASK 0
#define I2DAT_RW_MASK (Data_MASK)

/* I2ADR0 register bits */

/* I2SCLH register bits */

/* I2SCLL register bits */

/* I2CONCLR register bits */
#define AAC     2
#define SIC     3
#define STAC    5
#define I2ENC   6

#define AAC_MASK    (1 << AAC)
#define SIC_MASK    (1 << SIC)
#define STAC_MASK   (1 << STAC)
#define I2ENC_MASK  (1 << I2ENC)
#define I2CONCLR_RO_MASK 0
#define I2CONCLR_WO_MASK (AAC_MASK | SIC_MASK | STAC_MASK | I2ENC_MASK)
#define I2CONCLR_RW_MASK 0

/* I2MMCTRL register bits */
#define MM_ENA      0
#define MM_SCL      1
#define MATCH_ALL   2

/* I2ADR1 register bits */

/* I2ADR2 register bits */

/* I2ADR3 register bits */

/* I2DATA_BUFFER register bits */

/* I2MASK0 register bits */

/* I2MASK1 register bits */

/* I2MASK2 register bits */

/* I2MASK3 register bits */

/*------------- Inter IC Sound (I2S) -----------------------------------------*/
typedef struct
{
    vuint32_t I2SDAO;
    vuint32_t I2SDAI;
    vuint32_t I2STXFIFO;
    const vuint32_t I2SRXFIFO;
    const vuint32_t I2SSTATE;
    vuint32_t I2SDMA1;
    vuint32_t I2SDMA2;
    vuint32_t I2SIRQ;
    vuint32_t I2STXRATE;
    vuint32_t I2SRXRATE;
    vuint32_t I2STXBITRATE;
    vuint32_t I2SRXBITRATE;
    vuint32_t I2STXMODE;
    vuint32_t I2SRXMODE;
} LPC_I2S_TypeDef;

//TODO setup register bits

/*------------- Repetitive Interrupt Timer (RIT) -----------------------------*/
typedef struct
{
    vuint32_t RICOMPVAL;
    vuint32_t RIMASK;
    vuint8_t RICTRL;
        uint8_t RESERVED0[3];
    vuint32_t RICOUNTER;
} LPC_RIT_TypeDef;

/* RICOMPVAL register bits */
#define RICOMP 0

/* RIMASK register bits */
#define RIMASK 0

/* RICTRL register bits */
#define RITINT      0
#define RITENCLR    1
#define RITENBR     2
#define RITEN       3

#define RITINT_MASK     (1 << RITINT)
#define RITENCLR_MASK   (1 << RITENCLR)
#define RITENBR_MASK    (1 << RITENBR)
#define RITEN_MASK      (1 << RITEN)
#define RICTRL_RO_MASK   0
#define RICTRL_WO_MASK   0
#define RICTRL_RW_MASK   (RITINT_MASK | RITENCLR_MASK | RITENBR_MASK | RITEN_MASK)

/* RICOUNTER register bits */

/* 
 *
 *
 *
 *
 * Peripheral memory map 
 *
 * Values and offsets on pg. 131, LPC176X User Manual.
 *
 *
 */

// base addresses
#define LPC_SCS_BASE ( 0xE000E000UL ) // System Control Space (SCS), includes NVIC and SYSTICK

#define LPC_GPIO_BASE ( 0x2009C000UL )
#define LPC_APB0_BASE ( 0x40000000UL ) // Advanced Peripheral Busses (APB)
#define LPC_APB1_BASE ( 0x40080000UL )

/*                  */

// C-M3 Private Peripheral Bus
#define LPC_NVIC_BASE ( LPC_SCS_BASE + 0x100UL )

// APB0
#define LPC_TIM0_BASE  ( LPC_APB0_BASE + 0x04000)
#define LPC_TIM1_BASE  ( LPC_APB0_BASE + 0x08000)
#define LPC_UART0_BASE ( LPC_APB0_BASE + 0x0C000)
#define LPC_UART1_BASE ( LPC_APB0_BASE + 0x10000)
#define LPC_PWM1_BASE  ( LPC_APB0_BASE + 0x18000)
#define LPC_I2C0_BASE  ( LPC_APB0_BASE + 0x1C000)

#define LPC_GPIOINT_BASE ( LPC_APB0_BASE + 0x28080 )
#define LPC_PINCON_BASE  ( LPC_APB0_BASE + 0x2C000 )

#define LPC_ADC_BASE ( LPC_APB0_BASE + 0x34000 )

#define LPC_I2C1_BASE ( LPC_APB0_BASE + 0x5C000)

// APB1
#define LPC_TIM2_BASE ( LPC_APB1_BASE + 0x10000)
#define LPC_TIM3_BASE ( LPC_APB1_BASE + 0x14000)

#define LPC_I2C2_BASE   ( LPC_APB1_BASE + 0x20000)
#define LPC_I2S_BASE    ( LPC_APB1_BASE + 0x28000)
#define LPC_RIT_BASE    ( LPC_APB1_BASE + 0x30000 )

#define LPC_SC_BASE  ( LPC_APB1_BASE + 0x7C000 )

// GPIO
#define LPC_GPIO0_BASE ( LPC_GPIO_BASE + 0x00000 )
#define LPC_GPIO1_BASE ( LPC_GPIO_BASE + 0x00020 )
#define LPC_GPIO2_BASE ( LPC_GPIO_BASE + 0x00040 )
#define LPC_GPIO3_BASE ( LPC_GPIO_BASE + 0x00060 )
#define LPC_GPIO4_BASE ( LPC_GPIO_BASE + 0x00080 )

/*
 *
 *
 *
 * Peripheral Declaration
 *
 *
 *
 *
 */

#define LPC_NVIC ( (LPC_NVIC_TypeDef*) LPC_NVIC_BASE )

#define LPC_SC 		( (LPC_SC_TypeDef*) LPC_SC_BASE )
#define LPC_GPIO0 	( (LPC_GPIO_TypeDef*) LPC_GPIO0_BASE )
#define LPC_GPIO1 	( (LPC_GPIO_TypeDef*) LPC_GPIO1_BASE )
#define LPC_GPIO2 	( (LPC_GPIO_TypeDef*) LPC_GPIO2_BASE )
#define LPC_GPIO3 	( (LPC_GPIO_TypeDef*) LPC_GPIO3_BASE )
#define LPC_GPIO4 	( (LPC_GPIO_TypeDef*) LPC_GPIO4_BASE )

#define LPC_TIM0  ( (LPC_TIM_TypeDef*) LPC_TIM0_BASE)
#define LPC_TIM1  ( (LPC_TIM_TypeDef*) LPC_TIM1_BASE)
#define LPC_TIM2  ( (LPC_TIM_TypeDef*) LPC_TIM2_BASE)
#define LPC_TIM3  ( (LPC_TIM_TypeDef*) LPC_TIM3_BASE)
#define LPC_RIT   ( (LPC_RIT_TypeDef*) LPC_RIT_BASE )
#define LPC_UART0 ( (LPC_UART0_TypeDef*) LPC_UART0_BASE )
#define LPC_UART1 ( (LPC_UART1_TypeDef*) LPC_UART1_BASE )

#define LPC_PWM1 ( (LPC_PWM_TypeDef*) LPC_PWM1_BASE )
#define LPC_I2C0 ( (LPC_I2C_TypeDef*) LPC_I2C0_BASE )
#define LPC_I2C1 ( (LPC_I2C_TypeDef*) LPC_I2C1_BASE )
#define LPC_I2C2 ( (LPC_I2C_TypeDef*) LPC_I2C2_BASE )
#define LPC_I2S  ( (LPC_I2S_TypeDef*) LPC_I2S_BASE  )

#define LPC_GPIOINT ( (LPC_GPIOINT_TypeDef*) LPC_GPIOINT_BASE )
#define LPC_PINCON  ( (LPC_PINCON_TypeDef* ) LPC_PINCON_BASE  )

#define LPC_ADC ( (LPC_ADC_TypeDef*) LPC_ADC_BASE )

#endif
