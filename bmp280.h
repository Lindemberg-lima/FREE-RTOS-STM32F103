/**
 * @file    bmp280.h
 * @author  Lindemberg Roberto de Lima
 * @brief   BMP280 driver interface for STM32 using HAL I2C
 *
 * @details
 * This header file declares the public API, data structures, and
 * configuration macros for the BMP280 sensor driver.
 *
 * The driver is based on the official Bosch Sensortec BMP280 datasheet.
 * Temperature and pressure compensation algorithms follow the formulas
 * specified by Bosch and are required for proper sensor operation.
 *
 * The API design, STM32 HAL integration, and code organization were
 * developed by the author.
 *
 * @note
 * This is NOT an official Bosch Sensortec driver.
 *
 * @license
 * Free to use for educational, academic, and commercial purposes,
 * provided that this copyright notice and reference are preserved.
 *
 * Reference:
 * Bosch Sensortec BMP280 Datasheet
 */

#ifndef BMP280_H
#define BMP280_H

#include "stm32f1xx_hal.h"  // Change if using another stm32 series

/* I2C Address (SDO = GND -> 0x76, SDO = VDDIO -> 0x77) */
#define BMP280_I2C_ADDR        (0x76 << 1)

/* BMP280 Registers */
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5

/* Data registers */
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_PRESS_LSB     0xF8
#define BMP280_REG_PRESS_XLSB    0xF9

#define BMP280_REG_TEMP_MSB      0xFA
#define BMP280_REG_TEMP_LSB      0xFB
#define BMP280_REG_TEMP_XLSB     0xFC

/* Reset value */
#define BMP280_RESET_VAL       0xB6

/* Oversampling settings */
#define BMP280_OSRS_T_x1       (1 << 5)
#define BMP280_OSRS_P_x1       (1 << 2)
#define BMP280_MODE_NORMAL     (3)


typedef struct {
    int32_t dig_T1;
    int32_t dig_T2;
    int32_t dig_T3;
    int32_t dig_P1;
    int32_t dig_P2;
    int32_t dig_P3;
    int32_t dig_P4;
    int32_t dig_P5;
    int32_t dig_P6;
    int32_t dig_P7;
    int32_t dig_P8;
    int32_t dig_P9;
    int32_t t_fine;
} BMP280_CalibData_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    BMP280_CalibData_t calib;
} BMP280_Handle_t;

typedef struct {
    float temperature;
    float pressure;
} SensorData_t;

/* API */
HAL_StatusTypeDef BMP280_Init(BMP280_Handle_t *bmp);
HAL_StatusTypeDef BMP280_ReadTempPressure(BMP280_Handle_t *bmp, float *temperature, float *pressure);

#endif /* BMP280_H */
