/**
 * @file    mpu6050.c
 * @author  Lindemberg Roberto de Lima
 * @brief   MPU6050 driver for STM32 using HAL I2C
 *
 * @details
 * This driver was implemented based on the MPU6050 datasheet.
 * It provides basic device initialization and raw accelerometer
 * and gyroscope data reading via I2C.
 *
 * The driver structure, API design, and STM32 HAL integration
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

#include "mpu6050.h"

/* ================= mpu6050 Registers================= */

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define CONFIG_REG          0x1A
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C

#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

/* ================= Functions ================= */

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    /* Read WHO_AM_I */
    if (HAL_I2C_Mem_Read(hi2c,
                         MPU6050_ADDR,
                         WHO_AM_I_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &check,
                         1,
                         1000) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Check device ID */
    if (check != 0x68)
    {
        return HAL_ERROR;
    }

    /* Wake up the MPU6050 and disable the temperature sensor */
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c,
                      MPU6050_ADDR,
                      PWR_MGMT_1_REG,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      1000);

    /* Low-pass filter (DLPF = 5 Hz) */
    data = 0x06;
    HAL_I2C_Mem_Write(hi2c,
                      MPU6050_ADDR,
                      CONFIG_REG,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      1000);

    /* Accelerometer: ±2g */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c,
                      MPU6050_ADDR,
                      ACCEL_CONFIG_REG,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      1000);

    /* Gyroscope: ±250 °/s */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c,
                      MPU6050_ADDR,
                      GYRO_CONFIG_REG,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      1000);

    return HAL_OK;
}

/* ================================================= */

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
		                               readingAcel *reading)
{
    uint8_t recData[6];

    if (HAL_I2C_Mem_Read(hi2c,
                         MPU6050_ADDR,
                         ACCEL_XOUT_H_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         recData,
                         6,
                         1000) != HAL_OK)
    {
        return HAL_ERROR;
    }

    reading->accelX = (int16_t)(recData[0] << 8 | recData[1]);
    reading->accelY = (int16_t)(recData[2] << 8 | recData[3]);
    reading->accelZ = (int16_t)(recData[4] << 8 | recData[5]);

    return HAL_OK;
}

/* ================================================= */

HAL_StatusTypeDef MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
		                              readingGyro *reading)
{
    uint8_t recData[6];

    if (HAL_I2C_Mem_Read(hi2c,
                         MPU6050_ADDR,
                         GYRO_XOUT_H_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         recData,
                         6,
                         1000) != HAL_OK)
    {
        return HAL_ERROR;
    }

    reading->gyroX = (int16_t)(recData[0] << 8 | recData[1]);
    reading->gyroY = (int16_t)(recData[2] << 8 | recData[3]);
    reading->gyroZ = (int16_t)(recData[4] << 8 | recData[5]);

    return HAL_OK;
}
