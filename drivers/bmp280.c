/**
 * @file    bmp280.c
 * @author  Lindemberg Roberto de Lima
 * @brief   BMP280 driver for STM32 using HAL I2C
 * Date: January 29, 2026 
 * @details
 * This driver is based on the official Bosch Sensortec BMP280 datasheet.
 *
 * The temperature and pressure compensation algorithms are implemented
 * strictly according to the formulas provided by Bosch, which are required
 * for proper sensor operation.
 *
 * The driver structure, API design, STM32 HAL integration, and overall
 * code organization were developed by the author.
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

#include "bmp280.h"

/* =========================================================
   Low-level internal functions (I2C)
   ========================================================= */

/* Writes 1 byte to a register */
static HAL_StatusTypeDef BMP280_WriteReg(BMP280_Handle_t *bmp,
                                         uint8_t reg,
                                         uint8_t value)
{
    uint8_t txBuf[2];
    txBuf[0] = reg;
    txBuf[1] = value;

    return HAL_I2C_Master_Transmit(bmp->hi2c,
                                   BMP280_I2C_ADDR,
                                   txBuf,
                                   2,
                                   HAL_MAX_DELAY);
}

/* Reads N bytes from a register */
static HAL_StatusTypeDef BMP280_ReadRegs(BMP280_Handle_t *bmp,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t len)
{
    /* Send the register's address */
    if (HAL_I2C_Master_Transmit(bmp->hi2c,
                                BMP280_I2C_ADDR,
                                &reg,
                                1,
                                HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Read the data */
    return HAL_I2C_Master_Receive(bmp->hi2c,
                                  BMP280_I2C_ADDR,
                                  data,
                                  len,
                                  HAL_MAX_DELAY);
}

/* =========================================================
   Reading the calibration coefficients
   ========================================================= */
static HAL_StatusTypeDef BMP280_ReadCalibration(BMP280_Handle_t *bmp)
{
    uint8_t buf[24];

    /* Calibration records: 0x88 to 0xA1 */
    if (BMP280_ReadRegs(bmp, 0x88, buf, 24) != HAL_OK)
    {
        return HAL_ERROR;
    }

    bmp->calib.dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
    bmp->calib.dig_T2 = (int16_t) (buf[3]  << 8 | buf[2]);
    bmp->calib.dig_T3 = (int16_t) (buf[5]  << 8 | buf[4]);

    bmp->calib.dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
    bmp->calib.dig_P2 = (int16_t) (buf[9]  << 8 | buf[8]);
    bmp->calib.dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    bmp->calib.dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    bmp->calib.dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    bmp->calib.dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    bmp->calib.dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    bmp->calib.dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    bmp->calib.dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);

    return HAL_OK;
}

/* =========================================================
   BMP280 initialization
   ========================================================= */
HAL_StatusTypeDef BMP280_Init(BMP280_Handle_t *bmp)
{
    uint8_t id;

    /* ID reading */
    if (BMP280_ReadRegs(bmp, BMP280_REG_ID, &id, 1) != HAL_OK)
        return HAL_ERROR;

    if (id != 0x58)  /*  BMP280 ID */
        return HAL_ERROR;

    /* Software reset */
    if (BMP280_WriteReg(bmp, BMP280_REG_RESET, BMP280_RESET_VAL) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(100);

    /* Reading the coefficients */
    if (BMP280_ReadCalibration(bmp) != HAL_OK)
        return HAL_ERROR;

    /* Filter configuration and standby */
    uint8_t config = (0x00 << 5) |   // t_sb = 0.5 ms
                     (0x04 << 2) |   // IIR x16 filter
                     (0x00);         // disable SPI

    if (BMP280_WriteReg(bmp, BMP280_REG_CONFIG, config) != HAL_OK)
        return HAL_ERROR;

    /* Configuration: oversampling x1, normal mode */
    uint8_t ctrl_meas =
        BMP280_OSRS_T_x1 |
        BMP280_OSRS_P_x1 |
        BMP280_MODE_NORMAL;

    if (BMP280_WriteReg(bmp, BMP280_REG_CTRL_MEAS, ctrl_meas) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/* =========================================================
   Compensation algorithms (Bosch datasheet)
   ========================================================= */
static int32_t BMP280_CompensateTemp(BMP280_Handle_t *bmp, int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp->calib.dig_T1 << 1)))
            * ((int32_t)bmp->calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1))) >> 12) *
            ((int32_t)bmp->calib.dig_T3)) >> 14;

    bmp->calib.t_fine = var1 + var2;
    T = (bmp->calib.t_fine * 5 + 128) >> 8;

    return T; /* °C x100 */
}

static uint32_t BMP280_CompensatePressure(BMP280_Handle_t *bmp, int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)bmp->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->calib.dig_P6;
    var2 += (var1 * (int64_t)bmp->calib.dig_P5) << 17;
    var2 += ((int64_t)bmp->calib.dig_P4) << 35;

    var1 = ((var1 * var1 * (int64_t)bmp->calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp->calib.dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1) *
            ((int64_t)bmp->calib.dig_P1)) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)bmp->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->calib.dig_P7) << 4);

    return (uint32_t)p;
}

/* =========================================================
   Temperature and pressure reading
   ========================================================= */
HAL_StatusTypeDef BMP280_ReadTempPressure(BMP280_Handle_t *bmp,
                                          float *temperature,
                                          float *pressure)
{
    uint8_t data[6];

    if (BMP280_ReadRegs(bmp, BMP280_REG_PRESS_MSB, data, 6) != HAL_OK)
        return HAL_ERROR;

    int32_t adc_P = (int32_t)(
        ((uint32_t)data[0] << 12) |
        ((uint32_t)data[1] << 4)  |
        (data[2] >> 4));

    int32_t adc_T = (int32_t)(
        ((uint32_t)data[3] << 12) |
        ((uint32_t)data[4] << 4)  |
        (data[5] >> 4));

    int32_t t = BMP280_CompensateTemp(bmp, adc_T);
    uint32_t p = BMP280_CompensatePressure(bmp, adc_P);

    *temperature = t / 100.0f;
    *pressure    = (p / 256.0f) / 100.0f; /* hPa */

    return HAL_OK;
}


