#ifndef __QMC5883L_H
#define __QMC5883L_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define QMC5883L_I2C_ADDR         (0x0D << 1) // HAL dùng địa chỉ 8-bit (0x1A)

// Register Addresses
#define QMC5883L_REG_OUT_X_L      0x00
#define QMC5883L_REG_CONTROL_1    0x09
#define QMC5883L_REG_CONTROL_2    0x0A
#define QMC5883L_REG_SET_RESET    0x0B

// Control Register 1 settings
#define QMC5883L_MODE_STANDBY     0x00
#define QMC5883L_MODE_CONTINUOUS  0x01

#define QMC5883L_ODR_10HZ         0x00
#define QMC5883L_ODR_50HZ         0x04
#define QMC5883L_ODR_100HZ        0x08
#define QMC5883L_ODR_200HZ        0x0C

#define QMC5883L_RNG_2G           0x00
#define QMC5883L_RNG_8G           0x10

#define QMC5883L_OSR_512          0x00
#define QMC5883L_OSR_256          0x40
#define QMC5883L_OSR_128          0x80
#define QMC5883L_OSR_64           0xC0

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    
    // Internal state variables
    float magneticDeclinationDegrees;
    bool smoothUse;
    uint8_t smoothSteps;
    bool smoothAdvanced;
    
    int16_t vRaw[3];
    int16_t vCalibrated[3];
    int16_t vSmooth[3];
    
    int16_t vHistory[10][3];
    uint8_t vScan;
    int32_t vTotals[3];
    
    uint8_t rx[6];   // đệm đích của DMA (SRAM thường, không cache)

    float offset[3];
    float scale[3];
} QMC5883L_t;

// API Functions
void QMC5883L_Init(QMC5883L_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
void QMC5883L_SetMode(QMC5883L_t *dev, uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr);
void QMC5883L_SetMagneticDeclination(QMC5883L_t *dev, int degrees, uint8_t minutes);
void QMC5883L_SetSmoothing(QMC5883L_t *dev, uint8_t steps, bool adv);
void QMC5883L_Reset(QMC5883L_t *dev);

void QMC5883L_SetCalibrationOffsets(QMC5883L_t *dev, float x_off, float y_off, float z_off);
void QMC5883L_SetCalibrationScales(QMC5883L_t *dev, float x_scale, float y_scale, float z_scale);
void QMC5883L_SetCalibration(QMC5883L_t *dev, int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
void QMC5883L_ClearCalibration(QMC5883L_t *dev);
void QMC5883L_Calibrate(QMC5883L_t *dev);

HAL_StatusTypeDef QMC5883L_Read(QMC5883L_t *dev);          // blocking
HAL_StatusTypeDef QMC5883L_ReadStart_DMA(QMC5883L_t *dev); // non-blocking
int QMC5883L_OnRxCplt(QMC5883L_t *dev, I2C_HandleTypeDef *hi2c);

int16_t QMC5883L_GetX(QMC5883L_t *dev);
int16_t QMC5883L_GetY(QMC5883L_t *dev);
int16_t QMC5883L_GetZ(QMC5883L_t *dev);

int QMC5883L_GetAzimuth(QMC5883L_t *dev);
uint8_t QMC5883L_GetBearing(int azimuth);
void QMC5883L_GetDirection(char *dirArray, int azimuth);

#ifdef __cplusplus
}
#endif

#endif /* __QMC5883L_H */