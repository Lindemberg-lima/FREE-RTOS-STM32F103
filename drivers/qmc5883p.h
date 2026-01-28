/**
 * @file    qmc5883p.h
 * @author  Lindemberg Roberto de Lima
 * @brief   QMC5883P magnetometer driver interface for STM32 using HAL I2C
 *
 * @details
 * This header file declares the public API, data structures, and
 * configuration macros for the QMC5883P magnetometer driver.
 *
 * The driver is based on the QMC5883P datasheet and provides basic
 * sensor initialization and raw magnetic field data acquisition
 * (X, Y, Z axes) via I2C.
 *
 * The API design, STM32 HAL integration, and code organization
 * were developed by the author.
 *
 * @note
 * This is NOT an official QST (QMC) driver.
 *
 * @license
 * Free to use for educational, academic, and commercial purposes,
 * provided that this notice is preserved.
 *
 * Reference:
 * QMC5883P Datasheet (QST Corporation)
 */

#ifndef QMC5883P_H
#define QMC5883P_H

#include "stm32f1xx_hal.h"   // <-- change to your STM32 family
#include <stdint.h>

/* ================= I2C ADDRESS ================= */
//#define QMC5883P_I2C_ADDR     (0x0D << 1)   // HAL uses 8-bit address
#define QMC5883P_I2C_ADDR     (0x2C << 1)   // HAL uses 8-bit address

/* ================= REGISTERS =================== */
#define QMC5883P_REG_X_LSB    0x01
#define QMC5883P_REG_X_MSB    0x02
#define QMC5883P_REG_Y_LSB    0x03
#define QMC5883P_REG_Y_MSB    0x04
#define QMC5883P_REG_Z_LSB    0x05
#define QMC5883P_REG_Z_MSB    0x06
#define QMC5883P_REG_STATUS   0x09
#define QMC5883P_REG_CTRL1    0x0A
#define QMC5883P_REG_CTRL2    0x0B

/* ================= STATUS BITS ================= */
#define QMC5883P_STATUS_DRDY  0x01
#define QMC5883P_REG_SET_RESET  0x0B

/* CTRL1 */
#define QMC5883P_MODE_STANDBY 0x00
//#define QMC5883P_MODE_CONT    0x11
#define QMC5883P_MODE_CONT    0x01

#define QMC5883P_ODR_10HZ     (0 << 2)
#define QMC5883P_ODR_50HZ     (1 << 2)
#define QMC5883P_ODR_100HZ    (2 << 2)
#define QMC5883P_ODR_200HZ    (3 << 2)

#define QMC5883P_RNG_2G       (0 << 4)
#define QMC5883P_RNG_8G       (1 << 4)

#define QMC5883P_OSR_512      (3 << 6)
#define QMC5883P_OSR_256      (2 << 6)
#define QMC5883P_OSR_128      (1 << 6)
#define QMC5883P_OSR_64       (0 << 6)

/* ================= DATA STRUCT ================= */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} QMC5883P_Data_t;

/* ================= API ========================= */
HAL_StatusTypeDef QMC5883P_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef QMC5883P_Read(QMC5883P_Data_t *data);

#endif
