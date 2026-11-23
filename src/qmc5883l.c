#include "config.h"
#ifdef USE_MAG

#include "qmc5883l.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char bearings_table[16][3] = {
    {' ', ' ', 'N'}, {'N', 'N', 'E'}, {' ', 'N', 'E'}, {'E', 'N', 'E'},
    {' ', ' ', 'E'}, {'E', 'S', 'E'}, {' ', 'S', 'E'}, {'S', 'S', 'E'},
    {' ', ' ', 'S'}, {'S', 'S', 'W'}, {' ', 'S', 'W'}, {'W', 'S', 'W'},
    {' ', ' ', 'W'}, {'W', 'N', 'W'}, {' ', 'N', 'W'}, {'N', 'N', 'W'}
};

static void WriteReg(QMC5883L_t *dev, uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void ApplyCalibration(QMC5883L_t *dev) {
    dev->vCalibrated[0] = (int16_t)((dev->vRaw[0] - dev->offset[0]) * dev->scale[0]);
    dev->vCalibrated[1] = (int16_t)((dev->vRaw[1] - dev->offset[1]) * dev->scale[1]);
    dev->vCalibrated[2] = (int16_t)((dev->vRaw[2] - dev->offset[2]) * dev->scale[2]);
}

static void Smoothing(QMC5883L_t *dev) {
    uint8_t max_idx = 0;
    uint8_t min_idx = 0;
    
    if (dev->vScan > dev->smoothSteps - 1) {
        dev->vScan = 0;
    }
    
    for (int i = 0; i < 3; i++) {
        if (dev->vTotals[i] != 0) {
            dev->vTotals[i] -= dev->vHistory[dev->vScan][i];
        }
        dev->vHistory[dev->vScan][i] = dev->vCalibrated[i];
        dev->vTotals[i] += dev->vHistory[dev->vScan][i];
        
        if (dev->smoothAdvanced && dev->smoothSteps > 2) {
            max_idx = 0;
            for (int j = 0; j < dev->smoothSteps; j++) {
                if (dev->vHistory[j][i] > dev->vHistory[max_idx][i]) {
                    max_idx = j;
                }
            }
            
            min_idx = 0;
            for (int k = 0; k < dev->smoothSteps; k++) {
                if (dev->vHistory[k][i] < dev->vHistory[min_idx][i]) {
                    min_idx = k;
                }
            }
            
            dev->vSmooth[i] = (dev->vTotals[i] - (dev->vHistory[max_idx][i] + dev->vHistory[min_idx][i])) / (dev->smoothSteps - 2);
        } else {
            dev->vSmooth[i] = dev->vTotals[i] / dev->smoothSteps;
        }
    }
    
    dev->vScan++;
}

static int16_t GetAxis(QMC5883L_t *dev, int index) {
    if (dev->smoothUse) {
        return dev->vSmooth[index];
    }
    return dev->vCalibrated[index];
}

void QMC5883L_Init(QMC5883L_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    dev->hi2c = hi2c;
    dev->dev_addr = addr;
    dev->magneticDeclinationDegrees = 0.0f;
    dev->smoothUse = false;
    dev->smoothSteps = 5;
    dev->smoothAdvanced = false;
    dev->vScan = 0;
    
    for (int i = 0; i < 3; i++) {
        dev->vTotals[i] = 0;
        dev->offset[i] = 0.0f;
        dev->scale[i] = 1.0f;
    }
    
    WriteReg(dev, QMC5883L_REG_SET_RESET, 0x01);
    QMC5883L_SetMode(dev, QMC5883L_MODE_CONTINUOUS, QMC5883L_ODR_200HZ, QMC5883L_RNG_8G, QMC5883L_OSR_512);
}

void QMC5883L_SetMode(QMC5883L_t *dev, uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr) {
    WriteReg(dev, QMC5883L_REG_CONTROL_1, mode | odr | rng | osr);
}

void QMC5883L_SetMagneticDeclination(QMC5883L_t *dev, int degrees, uint8_t minutes) {
    dev->magneticDeclinationDegrees = (float)degrees + ((float)minutes / 60.0f);
}

void QMC5883L_SetSmoothing(QMC5883L_t *dev, uint8_t steps, bool adv) {
    dev->smoothUse = true;
    dev->smoothSteps = (steps > 10) ? 10 : steps;
    dev->smoothAdvanced = adv;
}

void QMC5883L_Reset(QMC5883L_t *dev) {
    WriteReg(dev, QMC5883L_REG_CONTROL_2, 0x80);
}

void QMC5883L_SetCalibrationOffsets(QMC5883L_t *dev, float x_off, float y_off, float z_off) {
    dev->offset[0] = x_off;
    dev->offset[1] = y_off;
    dev->offset[2] = z_off;
}

void QMC5883L_SetCalibrationScales(QMC5883L_t *dev, float x_scale, float y_scale, float z_scale) {
    dev->scale[0] = x_scale;
    dev->scale[1] = y_scale;
    dev->scale[2] = z_scale;
}

void QMC5883L_SetCalibration(QMC5883L_t *dev, int x_min, int x_max, int y_min, int y_max, int z_min, int z_max) {
    QMC5883L_SetCalibrationOffsets(dev, (float)(x_min + x_max) / 2.0f, (float)(y_min + y_max) / 2.0f, (float)(z_min + z_max) / 2.0f);

    float x_avg_delta = (float)(x_max - x_min) / 2.0f;
    float y_avg_delta = (float)(y_max - y_min) / 2.0f;
    float z_avg_delta = (float)(z_max - z_min) / 2.0f;

    float avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3.0f;

    QMC5883L_SetCalibrationScales(dev, avg_delta / x_avg_delta, avg_delta / y_avg_delta, avg_delta / z_avg_delta);
}

void QMC5883L_ClearCalibration(QMC5883L_t *dev) {
    QMC5883L_SetCalibrationOffsets(dev, 0.0f, 0.0f, 0.0f);
    QMC5883L_SetCalibrationScales(dev, 1.0f, 1.0f, 1.0f);
}

static void Decode(QMC5883L_t *dev) {
    const uint8_t *b = dev->rx;
    dev->vRaw[0] = (int16_t)(b[0] | (b[1] << 8));
    dev->vRaw[1] = (int16_t)(b[2] | (b[3] << 8));
    dev->vRaw[2] = (int16_t)(b[4] | (b[5] << 8));

    ApplyCalibration(dev);

    if (dev->smoothUse) {
        Smoothing(dev);
    }
}

/* Đọc blocking: chỉ dùng cho Calibrate/chẩn đoán, vòng lặp chính dùng ReadStart_DMA */
HAL_StatusTypeDef QMC5883L_Read(QMC5883L_t *dev) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, QMC5883L_REG_OUT_X_L, I2C_MEMADD_SIZE_8BIT, dev->rx, 6, 100);
    if (status == HAL_OK) {
        Decode(dev);
    }
    return status;
}

/* Non-blocking: trả về ngay, DMA đọc 6 byte, xong thì HAL gọi HAL_I2C_MemRxCpltCallback */
HAL_StatusTypeDef QMC5883L_ReadStart_DMA(QMC5883L_t *dev) {
    return HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->dev_addr, QMC5883L_REG_OUT_X_L, I2C_MEMADD_SIZE_8BIT, dev->rx, 6);
}

/* Gọi từ HAL_I2C_MemRxCpltCallback; trả 1 nếu gói này của QMC5883L đã được giải mã */
int QMC5883L_OnRxCplt(QMC5883L_t *dev, I2C_HandleTypeDef *hi2c) {
    if (hi2c != dev->hi2c) return 0;
    Decode(dev);
    return 1;
}

void QMC5883L_Calibrate(QMC5883L_t *dev) {
    QMC5883L_ClearCalibration(dev);
    int32_t calData[3][2] = {{32767, -32768}, {32767, -32768}, {32767, -32768}};
    
    uint32_t startTime = HAL_GetTick();

    while ((HAL_GetTick() - startTime) < 10000) {
        if (QMC5883L_Read(dev) == HAL_OK) {
            int16_t x = QMC5883L_GetX(dev);
            int16_t y = QMC5883L_GetY(dev);
            int16_t z = QMC5883L_GetZ(dev);

            if (x < calData[0][0]) calData[0][0] = x;
            if (x > calData[0][1]) calData[0][1] = x;

            if (y < calData[1][0]) calData[1][0] = y;
            if (y > calData[1][1]) calData[1][1] = y;

            if (z < calData[2][0]) calData[2][0] = z;
            if (z > calData[2][1]) calData[2][1] = z;
        }
        HAL_Delay(10);
    }

    QMC5883L_SetCalibration(dev, calData[0][0], calData[0][1], calData[1][0], calData[1][1], calData[2][0], calData[2][1]);
}

int16_t QMC5883L_GetX(QMC5883L_t *dev) { return GetAxis(dev, 0); }
int16_t QMC5883L_GetY(QMC5883L_t *dev) { return GetAxis(dev, 1); }
int16_t QMC5883L_GetZ(QMC5883L_t *dev) { return GetAxis(dev, 2); }

int QMC5883L_GetAzimuth(QMC5883L_t *dev) {
    float heading = atan2f((float)QMC5883L_GetY(dev), (float)QMC5883L_GetX(dev)) * 180.0f / M_PI;
    heading += dev->magneticDeclinationDegrees;
    
    int azimuth = (int)heading % 360;
    if (azimuth < 0) {
        azimuth += 360;
    }
    return azimuth;
}

uint8_t QMC5883L_GetBearing(int azimuth) {
    float a = (azimuth > -0.5f) ? (float)azimuth / 22.5f : ((float)azimuth + 360.0f) / 22.5f;
    float r = a - (int)a;
    uint8_t sexdec = (r >= 0.5f) ? (uint8_t)ceilf(a) : (uint8_t)floorf(a);
    return sexdec % 16;
}

void QMC5883L_GetDirection(char *dirArray, int azimuth) {
    uint8_t d = QMC5883L_GetBearing(azimuth);
    dirArray[0] = bearings_table[d][0];
    dirArray[1] = bearings_table[d][1];
    dirArray[2] = bearings_table[d][2];
}

#endif /* USE_MAG */
