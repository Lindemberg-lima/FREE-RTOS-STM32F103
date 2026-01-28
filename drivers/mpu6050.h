/**
 * @file    mpu6050.h
 * @author  Lindemberg Roberto de Lima
 * @brief   MPU6050 driver interface for STM32 using HAL I2C
 *
 * @details
 * This header file declares the public API, data structures, and
 * configuration macros for the MPU6050 sensor driver.
 *
 * The driver is based on the MPU6050 datasheet and provides basic
 * initialization as well as raw accelerometer and gyroscope data
 * acquisition via I2C.
 *
 * The API design, STM32 HAL integration, and code organization
 * were developed by the author.
 *
 * @note
 * This is NOT an official InvenSense driver.
 *
 * @license
 * Free to use for educational, academic, and commercial purposes,
 * provided that this notice is preserved.
 *
 * Reference:
 * InvenSense MPU6050 Datasheet
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ================= DEFINES ================= */

#define MPU6050_ADDR        (0x68 << 1)   //  I2C Address (HAL uses 8 bits)

//#define MPU6050_ADDR   0xD0
/* ================= Types ================= */

typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} readingAcel;

typedef struct
{
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} readingGyro;

typedef struct readingImu { 
	float imugyroX;
	float imugyroY;
	float imugyroZ;
	float imuaccelX;
	float imuaccelY;
	float imuaccelZ;
} readingImu;

/* ================= API ================= */

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
		                              readingAcel *reading);

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
		                             readingGyro *reading);

#endif /* MPU6050_H */
