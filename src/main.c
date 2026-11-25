#include "main.h"
#include "icm45686_fc.h"
#ifdef USE_BARO
#include "bmp388.h"
#endif

#define IMU_FS_HZ 1600.0f   /* phải khớp gyro_odr trong board.c (GYRO_ODR_1600_HZ) */
#define DEG2RAD   0.0174532925f

volatile int      g_imu_rc = -1;   /* 0 = OK, ICM45686_ERR_WHOAMI = không nhận được WHO_AM_I */ 
volatile uint8_t  g_imu_whoami = 0;   /* WHO_AM_I đọc được từ ICM45686 */
volatile int      g_calib_rc = -1;   /* 0 = OK, ICM45686_ERR_MOVING = gyro không đứng yên */
volatile float    g_dt = 1.0f / IMU_FS_HZ;  /* dt (s) của mẫu vừa xử lý, từ timestamp FIFO */
 volatile int      g_bmp_rc = -1;   /* 0 = OK, BMP388_ERR_WHOAMI = không nhận được WHO_AM_I */
 volatile uint8_t  g_bmp_whoami = 0;   /* WHO_AM_I đọc được từ BMP388 */
Mahony_t          mahony;
volatile uint32_t g_imu_hz;   /* mẫu/giây main nhận được: xem trong debugger */
volatile int g_bmp_hz = 0;   /* mẫu/giây main nhận được: xem trong debugger */

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;
Attitude_t attitude;

typedef struct {
    float temp;
    float pressure;
    float altitude;  
} Baro_t;
Baro_t baro;

typedef struct{
    float accel[3];
    float gyro[3];
    float mag[3];
} Filter_t;

Biquad lpf_gyro[3];
Biquad lpf_accel[3];
Filter_t filtered;

void IMU_FilterInit(void){
    for(int i = 0; i < 3; i++){
        Biquad_Init(&lpf_gyro[i]);
        Biquad_Init(&lpf_accel[i]);
        Biquad_LowPassSetup(&lpf_gyro[i],  40.0f, 0.707f, IMU_FS_HZ);
        Biquad_LowPassSetup(&lpf_accel[i], 40.0f, 0.707f, IMU_FS_HZ);
    }
}
extern TIM_HandleTypeDef htim5;
static inline uint32_t Micros(void){
    return __HAL_TIM_GET_COUNTER(&htim5);
}

#if defined(USE_MAG) || defined(USE_BARO)
typedef enum {
    I2C1_OWNER_NONE = 0,
    I2C1_OWNER_MAG,
    I2C1_OWNER_BARO
} i2c1_owner_t;
static volatile i2c1_owner_t i2c1_owner = I2C1_OWNER_NONE;
#endif

#ifdef USE_MAG
QMC5883L_t mag;
volatile uint8_t  mag_busy = 0, mag_new = 0;
volatile uint32_t mag_err = 0;
#endif

int main(void)
{
    icm45686_sample_t imu_s;
    bmp388_sample_t bmp_s;

    HAL_Init();             /* SysTick 1 ms, NVIC priority grouping */
    HAL_ICACHE_Enable();
    SystemClock_Config();   /* 250 MHz từ HSE 8 MHz + PLL1 */
    GPIOS_Init();
    SPI1_Init();
    TIMER5_Init();
    I2C1_Init();
    USART1_Init();
    MAVLINK_Init();
#ifdef USE_MAG
    QMC5883L_Init(&mag, &hi2c1, QMC5883L_I2C_ADDR);
#endif
#ifdef USE_GPS
    USART3_Init();
    GPS_Init(&huart3);
#endif
    ICM45686_Init();
    BMP388_Init();
    g_imu_rc = ICM45686_Init();
    
    /* Đọc WHO_AM_I cả khi init lỗi để chẩn đoán (transport chỉ có sau khi init đi tới đó) */
    if (imu.dev.transport.read_reg != NULL)
        inv_imu_get_who_am_i(&imu.dev, (uint8_t *)&g_imu_whoami);
    g_bmp_rc = BMP388_Init();
    g_bmp_whoami = bmp.chip_id;
    if (g_imu_rc != ICM45686_OK)
    {
        Error_Handler();    /* dừng tại đây: xem g_imu_rc, g_imu_whoami trong debugger */
    }

    for (int try = 0; try < 5; try++) {   /* retry: lần đầu có thể trùng lúc vừa cắm/đặt board */
        g_calib_rc = icm45686_calibrate_gyro(&imu, 500, 2.0f);
        if (g_calib_rc != ICM45686_ERR_MOVING) break;
        HAL_Delay(500);
    }
    if (g_calib_rc != ICM45686_OK)
    {
        Error_Handler();    /* gyro rung hoặc đọc lỗi: xem g_calib_rc trong debugger */
    }
#ifdef USE_BARO
    bmp388_calibrate_ground(&bmp, 50);
    baro.temp     = bmp.last.temperature_c;
    baro.pressure = bmp.last.pressure_pa;
    baro.altitude = 0.0f;
#endif

    IMU_FilterInit();
    Mahony_Init(&mahony);
    icm45686_start(&imu);
    
    uint32_t hz_t0 = Micros(), hz_cnt = 0;
    uint32_t mav_t = Micros();
#ifdef USE_BARO
    uint32_t baro_t0 = Micros(), baro_cnt = 0;
#endif
#ifdef USE_MAG
    uint32_t mag_t0 = Micros();
#endif
    while (1)
    {
        mavlink_poll(&mav);
        while(icm45686_pop(&imu, &imu_s))
        {
            hz_cnt++;
            /* dt từ timestamp FIFO; 0 hoặc lệch xa danh định (mất frame, timestamp chưa bật) thì dùng danh định */
            float dt_att = (float)imu_s.dt_us * 1e-6f;
            if (imu_s.dt_us == 0 || dt_att > 4.0f / IMU_FS_HZ) dt_att = 1.0f / IMU_FS_HZ;
            g_dt = dt_att;

            for (int i = 0; i < 3; i++) {
                filtered.gyro[i]  = Biquad_Update(&lpf_gyro[i],  imu_s.gyro_dps[i]);
                filtered.accel[i] = Biquad_Update(&lpf_accel[i], imu_s.accel_g[i]);
            }
#ifdef USE_MAG
            if (Micros() - mag_t0 >= 20000u) {  /* 50 Hz khớp QMC5883L_ODR_50_HZ */
                mag_t0 += 20000u;
                if (i2c1_owner == I2C1_OWNER_NONE && !mag_busy) {
                    i2c1_owner = I2C1_OWNER_MAG;
                    if (QMC5883L_ReadStart_DMA(&mag) == HAL_OK) {
                        mag_busy = 1;
                    } else {
                        i2c1_owner = I2C1_OWNER_NONE;
                    }
                }
            }
            filtered.mag[0] = (float)QMC5883L_GetX(&mag);
            filtered.mag[1] = (float)QMC5883L_GetY(&mag);
            filtered.mag[2] = (float)QMC5883L_GetZ(&mag);
            Mahony_Update(&mahony,
                          filtered.gyro[0] * DEG2RAD, filtered.gyro[1] * DEG2RAD, filtered.gyro[2] * DEG2RAD,
                          filtered.accel[0], filtered.accel[1], filtered.accel[2],
                          filtered.mag[0], filtered.mag[1], filtered.mag[2], dt_att);
#else
            Mahony_UpdateIMU(&mahony,
                             filtered.gyro[0] * DEG2RAD, filtered.gyro[1] * DEG2RAD, filtered.gyro[2] * DEG2RAD,
                             filtered.accel[0], filtered.accel[1], filtered.accel[2], dt_att);
#endif
        }
        attitude.roll  = Mahony_GetRoll(&mahony);
        attitude.pitch = Mahony_GetPitch(&mahony);
        attitude.yaw   = Mahony_GetYaw(&mahony);

        if (Micros() - hz_t0 >= 1000000u) {   /* mỗi 1 s: số mẫu main nhận phải ≈ IMU_FS_HZ */
            g_imu_hz = hz_cnt; hz_cnt = 0; hz_t0 += 1000000u;
#ifdef USE_BARO
            g_bmp_hz = (int)baro_cnt; baro_cnt = 0;
#endif
            mavlink_send_heartbeat(&mav);
        }

        if ((Micros() - mav_t) >= 50000u) {   /* mỗi 50 ms (20 Hz): gửi ATTITUDE + BARO/HUD */
            mav_t += 50000u;
            mavlink_send_attitude(&mav,
                                  Mahony_GetRollRadians(&mahony),
                                  Mahony_GetPitchRadians(&mahony),
                                  Mahony_GetYawRadians(&mahony),
                                  filtered.gyro[0] * DEG2RAD,
                                  filtered.gyro[1] * DEG2RAD,
                                  filtered.gyro[2] * DEG2RAD);
#ifdef USE_BARO
            mavlink_send_scaled_pressure(&mav, baro.pressure, baro.temp);
            mavlink_send_vfr_hud(&mav, 0.0f, 0.0f, (int16_t)attitude.yaw, 0u, baro.altitude, 0.0f);
#endif
            mavlink_poll(&mav); /* kích truyền cả cụm 3 gói trong 1 lần DMA duy nhất */
        }

#ifdef USE_BARO
        if ((Micros() - baro_t0) >= 20000u) {  /* 50 Hz khớp BMP388_ODR_50_HZ */
            baro_t0 += 20000u;
            if (i2c1_owner == I2C1_OWNER_NONE) {
                i2c1_owner = I2C1_OWNER_BARO;
                if (bmp388_trigger_read(&bmp) != BMP388_OK) {
                    i2c1_owner = I2C1_OWNER_NONE;
                }
            }
        }
        if (bmp388_get_latest(&bmp, &bmp_s)) {
            baro.temp     = bmp_s.temperature_c;
            baro.pressure = bmp_s.pressure_pa;
            baro.altitude = bmp388_altitude_m(&bmp, bmp_s.pressure_pa);
            baro_cnt++;
        }
#endif

#ifdef USE_GPS
        if (GPS_Poll()) {   /* kết quả trong global gps */
            mavlink_on_new_gps_fix(&mav);
        }
#endif
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) icm45686_on_drdy(&imu);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == imu.bus.hspi) icm45686_on_dma_complete(&imu);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == imu.bus.hspi) icm45686_on_dma_error(&imu);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if defined(USE_MAG) || defined(USE_BARO)
    if (hi2c != &hi2c1) return;
    i2c1_owner_t owner = i2c1_owner;
    i2c1_owner = I2C1_OWNER_NONE;
#ifdef USE_BARO
    if (owner == I2C1_OWNER_BARO) {
        bmp388_on_xfer_complete(&bmp);
        return;
    }
#endif
#ifdef USE_MAG
    if (owner == I2C1_OWNER_MAG && QMC5883L_OnRxCplt(&mag, hi2c)) {
        mag_busy = 0;
        mag_new  = 1;
    }
#endif
    (void)owner;
#else
    (void)hi2c;
#endif
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
#if defined(USE_MAG) || defined(USE_BARO)
    if (hi2c != &hi2c1) return;
    i2c1_owner_t owner = i2c1_owner;
    i2c1_owner = I2C1_OWNER_NONE;
#ifdef USE_BARO
    if (owner == I2C1_OWNER_BARO) {
        bmp388_on_xfer_error(&bmp);
        return;
    }
#endif
#ifdef USE_MAG
    if (owner == I2C1_OWNER_MAG) {
        mag_busy = 0;
        mag_err++;
    }
#endif
    (void)owner;
#else
    (void)hi2c;
#endif
}

#ifdef USE_GPS
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) { GPS_OnRxEvent(huart, size); }
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    mavlink_on_tx_complete(&mav, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    mavlink_on_tx_error(&mav, huart);
#ifdef USE_GPS
    GPS_OnError(huart);
#endif
}
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {  
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif

