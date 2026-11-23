#include "main.h"
#include "icm45686_fc.h"

#define IMU_FS_HZ 1600.0f   /* phải khớp gyro_odr trong board.c (GYRO_ODR_1600_HZ) */
#define DEG2RAD   0.0174532925f

volatile int      g_calib_rc = -1;   /* 0 = OK, ICM45686_ERR_MOVING = gyro không đứng yên */
volatile float    g_dt = 1.0f / IMU_FS_HZ;  /* dt (s) của mẫu vừa xử lý, từ timestamp FIFO */
Mahony_t          mahony;
volatile uint32_t g_imu_hz;   /* mẫu/giây main nhận được: xem trong debugger */
volatile float Ax = 0;
volatile float Ay = 0;
volatile float Az = 0;
volatile float Gx = 0;
volatile float Gy = 0;
volatile float Gz = 0;
volatile int     g_imu_rc     = -1;  /* 0 = OK, -100 = sai WHO_AM_I, khác = lỗi SPI/TDK */
volatile uint8_t g_imu_whoami = 0;   /* kỳ vọng INV_IMU_WHOAMI (0xE9) */

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;
Attitude_t attitude;

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

#ifdef USE_MAG
QMC5883L_t mag;
volatile uint8_t  mag_busy = 0, mag_new = 0;
volatile uint32_t mag_err = 0;
#endif

int main(void)
{
    icm45686_sample_t s;

    HAL_Init();             /* SysTick 1 ms, NVIC priority grouping */
    SystemClock_Config();   /* 250 MHz từ HSE 8 MHz + PLL1 */
    GPIOS_Init();
    SPI1_Init();
    TIMER5_Init();
#ifdef USE_MAG
    I2C1_Init();
    QMC5883L_Init(&mag, &hi2c1, QMC5883L_I2C_ADDR);
#endif
#ifdef USE_GPS
    USART3_Init();
    GPS_Init(&huart3);
#endif

    g_imu_rc = ICM45686_Init();
    /* Đọc WHO_AM_I cả khi init lỗi để chẩn đoán (transport chỉ có sau khi init đi tới đó) */
    if (imu.dev.transport.read_reg != NULL)
        inv_imu_get_who_am_i(&imu.dev, (uint8_t *)&g_imu_whoami);
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
    IMU_FilterInit();
    Mahony_Init(&mahony);
    icm45686_start(&imu);
    
    uint32_t hz_t0 = Micros(), hz_cnt = 0;
    while (1)
    {
        while(icm45686_pop(&imu, &s))
        {
            hz_cnt++;
            Ax = s.accel_g[0];
            Ay = s.accel_g[1];
            Az = s.accel_g[2];
            Gx = s.gyro_dps[0];
            Gy = s.gyro_dps[1];
            Gz = s.gyro_dps[2];

            /* dt từ timestamp FIFO; 0 hoặc lệch xa danh định (mất frame, timestamp chưa bật) thì dùng danh định */
            float dt = (float)s.dt_us * 1e-6f;
            if (s.dt_us == 0 || dt > 4.0f / IMU_FS_HZ) dt = 1.0f / IMU_FS_HZ;
            g_dt = dt;

            for (int i = 0; i < 3; i++) {
                filtered.gyro[i]  = Biquad_Update(&lpf_gyro[i],  s.gyro_dps[i]);
                filtered.accel[i] = Biquad_Update(&lpf_accel[i], s.accel_g[i]);
            }
            Mahony_UpdateIMU(&mahony,
                             filtered.gyro[0] * DEG2RAD, filtered.gyro[1] * DEG2RAD, filtered.gyro[2] * DEG2RAD,
                             filtered.accel[0], filtered.accel[1], filtered.accel[2], dt);
        }
        attitude.roll  = Mahony_GetRoll(&mahony);
        attitude.pitch = Mahony_GetPitch(&mahony);
        attitude.yaw   = Mahony_GetYaw(&mahony);
        if (Micros() - hz_t0 >= 1000000u) {   /* mỗi 1 s: số mẫu main nhận phải ≈ IMU_FS_HZ */
            g_imu_hz = hz_cnt; hz_cnt = 0; hz_t0 += 1000000u;
        }
#ifdef USE_MAG
        if (!mag_busy && QMC5883L_ReadStart_DMA(&mag) == HAL_OK) mag_busy = 1;
#endif
#ifdef USE_GPS
        GPS_Poll();   /* kết quả trong global gps */
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
#ifdef USE_MAG
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (QMC5883L_OnRxCplt(&mag, hi2c)) { mag_busy = 0; mag_new = 1; }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == mag.hi2c) { mag_busy = 0; mag_err++; }
}
#endif

#ifdef USE_GPS
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) { GPS_OnRxEvent(huart, size); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)                    { GPS_OnError(huart); }
#endif

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

