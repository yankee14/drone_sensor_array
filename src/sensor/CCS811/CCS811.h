/*
===============================================================================
 Name        : CCS811.h
 Author      : Ed Peguillan III
 Version     : R0.1a
 Description : CO2 and VOC sensor register addresses, bit names, and functions
===============================================================================
*/

#ifndef _SENSOR_CCS811_H_
#define _SENSOR_CCS811_H_

#include "../../types.h"

#define CCS811_I2C_ADDR_PRI 0x5A
#define CCS811_I2C_ADDR_SEC 0x5B

/* STATUS register */
#define CCS811_STATUS_ADDR      0x00 // RO
#define CCS811_STATUS_NUM_BYTES 1
/* STATUS register bits */
#define CCS811_STATUS_ERROR        0
#define CCS811_STATUS_DATA_READY   3
#define CCS811_STATUS_APP_VALID    4
#define CCS811_STATUS_FW_MODE      7
/* STATUS register masks */
#define CCS811_STATUS_ERROR_MASK       (1 << CCS811_STATUS_ERROR)
#define CCS811_STATUS_DATA_READY_MASK  (1 << CCS811_STATUS_DATA_READY)
#define CCS811_STATUS_APP_VALID_MASK   (1 << CCS811_STATUS_APP_VALID)
#define CCS811_STATUS_FW_MODE_MASK     (1 << CCS811_STATUS_FW_MODE)
#define CCS811_STATUS_RO_MASK   (CCS811_STATUS_ERROR_MASK | CCS811_STATUS_DATA_READY_MASK | CCS811_STATUS_APP_VALID_MASK | CCS811_STATUS_FW_MODE_MASK)
#define CCS811_STATUS_WO_MASK   0
#define CCS811_STATUS_RW_MASK   0

/* MEAS_MODE register */
#define CCS811_MEAS_MODE_ADDR       0x01 // RW
#define CCS811_MEAS_MODE_NUM_BYTES  1
/* MEAS_MODE register bits */
#define CCS811_INT_THRESH           2
#define CCS811_INT_DATARDY          3
#define CCS811_DRIVE_MODE           4
#define CCS811_INT_THRESH_DISABLE   0
#define CCS811_INT_THRESH_ENABLE    1
#define CCS811_INT_DATARDY_DISABLE  0
#define CCS811_INT_DATARDY_ENABLE   1
#define CCS811_DRIVE_MODE_0         0b000 // idle
#define CCS811_DRIVE_MODE_1         0b001 // constant power, 1 Hz
#define CCS811_DRIVE_MODE_2         0b010 // pulse heating, 1/10 Hz
#define CCS811_DRIVE_MODE_3         0b011 // low power pulse heating, 1/60 Hz
#define CCS811_DRIVE_MODE_4         0b100 // constant power, 4 Hz (ALG_RESULT_DATA register disabled)
/* MEAS_MODE register masks */
#define CCS811_INT_THRESH_MASK      (1 << CCS811_INT_THRESH)
#define CCS811_INT_DATARDY_MASK     (1 << CCS811_INT_DATARDY)
#define CCS811_DRIVE_MODE_MASK      (0b111 << CCS811_DRIVE_MODE)
#define CCS811_MEAS_MODE_RO_MASK    0
#define CCS811_MEAS_MODE_WO_MASk    0
#define CCS811_MEAS_MODE_RW_MASK    (CCS811_INT_THRESH_MASK | CCS811_INT_DATARDY_MASK | CCS811_DRIVE_MODE_MASK)

/* ALG_RESULT_DATA register */
#define CCS811_ALG_RESULT_DATA_ADDR         0x02 // RO
#define CCS811_ALG_RESULT_DATA_NUM_BYTES    8
/* ALG_RESULT_DATA register bits */
#define CCS811_ALG_EC02_HIGH            0   // byte 0
#define CCS811_ALG_EC02_LOW             8   // byte 1
#define CCS811_ALG_TVOC_HIGH            16
#define CCS811_ALG_TVOC_LOW             24
#define CCS811_ALG_STATUS               32
#define CCS811_ALG_ERROR_ID             40
#define CCS811_ALG_RAW_DATA_HIGH        48
#define CCS811_ALG_RAW_DATA_LOW         56  // byte 7
/* ALG_RESULT_DATA register masks */

/* RAW_DATA register */
#define CCS811_RAW_DATA_ADDR            0x03 // RO
#define CCS811_RAW_DATA_NUM_BYTES       2
/* RAW_DATA register bits */
#define CCS811_RAW_CURRENT              2
#define CCS811_RAW_VOLTAGE_HIGH         0
#define CCS811_RAW_VOLTAGE_LOW          0
/* RAW_DATA register masks*/
#define CCS811_RAW_CURRENT_MASK         (0b111111 << CCS811_RAW_CURRENT)
#define CCS811_RAW_VOLTAGE_HIGH_MASK    (0b11 << CCS811_RAW_VOLTAGE_HIGH)
#define CCS811_RAW_VOLTAGE_LOW_MASK     (0xFF << CCS811_RAW_VOLTAGE_LOW)

/* ENV_DATA register */
#define CCS811_ENV_DATA_ADDR        0x05 // WO
#define CCS811_ENV_DATA_NUM_BYTES   4
/* ENV_DATA register bits */
#define CCS811_ENV_HUMIDITY             1
#define CCS811_ENV_HUMIDITY_PERC_HIGH   0 // byte 0
#define CCS811_ENV_HUMIDITY_PERC_LOW    0 // byte 1
#define CCS811_ENV_TEMP                 1
#define CCS811_ENV_TEMP_PERC_HIGH       0 // byte 2
#define CCS811_ENV_TEMP_PERC_LOW        0 // byte 3
/* ENV_DATA register masks */
#define CCS811_ENV_HUMIDITY_MASK            (0x7F << CCS811_ENV_HUMIDITY)
#define CCS811_ENV_HUMIDITY_PERC_HIGH_MASK  (0b1 << CCS811_ENV_HUMIDITY_PERC_HIGH)
#define CCS811_ENV_HUMIDITY_PERC_LOW_MASK   (0xFF << CCS811_ENV_HUMIDITY_PERC_LOW)
#define CCS811_ENV_TEMP_MASK                (0x7F << CCS811_ENV_TEMP)
#define CCS811_ENV_TEMP_PERC_HIGH_MASK      (0b1 << CCS811_ENV_TEMP_PERC_HIGH)
#define CCS811_ENV_TEMP_PERC_LOW_MASK       (0xFF << CCS811_ENV_TEMP_PERC_LOW)

/* NTC register */
#define CCS811_NTC_ADDR             0x06 // RO
#define CCS811_NTC_NUM_BYTES        4
/* NTC register bits */
#define CCS811_NTC_VRref_HIGH       0 // byte 0
#define CCS811_NTC_VRref_LOW        0 // byte 1
#define CCS811_NTC_VRntc_HIGH       0 // byte 2
#define CCS811_NTC_VRntc_LOW        0 // byte 3
/* NTC register masks */

/* HW_ID register */
#define CCS811_HW_ID_ADDR           0x20 // RO
#define CCS811_HW_ID_NUM_BYTES      1
/* HW_ID register bits */
#define CCS811_HW_ID                0
/* HW_ID register masks */
#define CCS811_HW_ID_MASK           0x81 // HW ID for compare

/* ERROR_ID register */
#define CCS811_ERROR_ID_ADDR        0xE0 // RO
#define CCS811_ERROR_ID_NUM_BYTES   1
/* ERROR_ID register bits */
#define CCS811_ERROR_WRITE_REG_INVALID  0
#define CCS811_ERROR_READ_REG_INVALID   1
#define CCS811_ERROR_MEASMODE_INVALID   2
#define CCS811_ERROR_MAX_RESISTANCE     3
#define CCS811_ERROR_HEATER_FAULT       4
#define CCS811_ERROR_HEATER_SUPPLY      5
/* ERROR_ID register masks*/
#define CCS811_ERROR_WRITE_REG_INVALID_MASK (1 << CCS811_ERROR_WRITE_REG_INVALID)
#define CCS811_ERROR_READ_REG_INVALID_MASK  (1 << CCS811_ERROR_READ_REG_INVALID)
#define CCS811_ERROR_MEASMODE_INVALID_MASK  (1 << CCS811_ERROR_MEASMODE_INVALID)
#define CCS811_ERROR_MAX_RESISTANCE_MASK    (1 << CCS811_ERROR_MAX_RESISTANCE)
#define CCS811_ERROR_HEATER_FAULT_MASK      (1 << CCS811_ERROR_HEATER_FAULT)
#define CCS811_ERROR_HEATER_SUPPLY_MASK     (1 << CCS811_ERROR_HEATER_SUPPLY)

/* APP_START register */
#define CCS811_APP_START_ADDR       0xF4 // WO
#define CCS811_APP_START_NUM_BYTES  1
/* APP_START register bits */
#define CCS811_APP_START
/* APP_START register masks*/

/* getters */
uint32_t get_STATUS(void);
uint32_t get_MEAS_MODE(void);
uint32_t* get_ALG_RESULT_DATA(uint32_t* data, uint32_t numBytes);
uint32_t* get_RAW_DATA(uint32_t* data);
uint32_t* get_NTC(uint32_t* data);
uint32_t get_HW_ID(void);
uint32_t get_ERROR_ID(void);

/* setters */
void set_MEAS_MODE(uint32_t data);
void set_ENV_DATA(uint32_t* data);

#endif
