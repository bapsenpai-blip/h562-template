#ifndef CONFIG_H_
#define CONFIG_H_

/* Comment dòng nào để tắt cảm biến đó: driver .c tương ứng không được biên dịch. */
//#define USE_GPS   /* u-blox UBX NAV-PVT qua USART3 (PB10 TX / PB11 RX) + GPDMA1 CH1 *///
//#define USE_MAG   /* QMC5883L qua I2C1 (PB6 SCL / PB7 SDA) + GPDMA1 CH0 */
#define USE_BARO  /* BMP388 qua I2C1 (PB6 SCL / PB7 SDA) + GPDMA1 CH0 */

#endif
