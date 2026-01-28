/**
 * @file    qmc5883p.c
 * @author  Lindemberg Roberto de Lima
 * @brief   QMC5883P magnetometer driver for STM32 using HAL I2C
 *
 * @details
 * This driver was implemented based on the QMC5883P datasheet.
 * It provides basic sensor initialization and raw magnetic field
 * data acquisition (X, Y, Z axes) via I2C.
 *
 * The driver includes the mandatory SET/RESET period configuration
 * required for proper sensor operation, as specified by the manufacturer.
 *
 * The driver structure, API design, STM32 HAL integration, and
 * code organization were developed by the author.
 *
 * @note
 * This is NOT an official QST (QMC) driver.
 *
 * @license
 * Free to use for educational, academic, and commercial purposes,
 * provided that this notice and reference are preserved.
 *
 * Reference:
 * QMC5883P Datasheet (QST Corporation)
 */

#include "qmc5883p.h"

/* Pointer to the I2C handle used by the driver */
static I2C_HandleTypeDef *qmc_i2c;

/* ================= LOW-LEVEL I2C ================= */

/*
 * Writes one byte to a register
 */
static HAL_StatusTypeDef qmc_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg;
    tx_buf[1] = data;

    return HAL_I2C_Master_Transmit(qmc_i2c,
                                   QMC5883P_I2C_ADDR,
                                   tx_buf,
                                   2,
                                   HAL_MAX_DELAY);
}

/*
 * Reads N bytes starting from a register
 */
static HAL_StatusTypeDef qmc_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Send register address */
    if (HAL_I2C_Master_Transmit(qmc_i2c,
                                QMC5883P_I2C_ADDR,
                                &reg,
                                1,
                                HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Read data */
    return HAL_I2C_Master_Receive(qmc_i2c,
                                  QMC5883P_I2C_ADDR,
                                  buf,
                                  len,
                                  HAL_MAX_DELAY);
}

/* ================= DRIVER API ================= */

HAL_StatusTypeDef QMC5883P_Init(I2C_HandleTypeDef *hi2c)
{
    qmc_i2c = hi2c;

    /* ---------- Soft reset ---------- */
    if (qmc_write(QMC5883P_REG_CTRL2, 0x80) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    /* -------- SET/RESET period (MANDATORY) -------- */
    if (qmc_write(QMC5883P_REG_SET_RESET, 0x01) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    /* ---------- Configuration ---------- */
    /*
     * Continuous mode
     * ODR = 200 Hz
     * Range = ±8G
     * OSR = 512
     */
    uint8_t ctrl1 = QMC5883P_MODE_CONT |
                    QMC5883P_ODR_200HZ |
                    QMC5883P_RNG_8G |
                    QMC5883P_OSR_512;

    if (qmc_write(QMC5883P_REG_CTRL1, ctrl1) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_Read(QMC5883P_Data_t *data)
{
    uint8_t buf[6];

    /* Direct read of X, Y, Z registers */
    if (qmc_read(QMC5883P_REG_X_LSB, buf, 6) != HAL_OK)
        return HAL_ERROR;

    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);

    return HAL_OK;
}
