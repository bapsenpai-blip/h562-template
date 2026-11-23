#include "config.h"
#ifdef USE_GPS

#include "ubx_gps.h"
#include <string.h>

/* ------------------------------------------------------------------
   UART RX bằng GPDMA + ngắt IDLE: DMA đổ vào dma_buf, khi đường truyền rảnh
   (hoặc đầy buffer) HAL gọi RxEvent -> chép vào ring buffer rồi bật DMA lại.
   Parser UBX chạy trong vòng lặp main.
   shortcut: DMA normal-mode, restart trong ISR mất vài chục us không nhận byte,
   ở 115200 baud/5 Hz không sao; nếu mất gói thì chuyển sang circular linked-list.
   ------------------------------------------------------------------ */

#define RING_SIZE  512u                 /* phải là lũy thừa của 2 */
#define RING_MASK  (RING_SIZE - 1u)

#define UBX_SYNC1        0xB5
#define UBX_SYNC2        0x62
#define UBX_CLASS_NAV    0x01
#define UBX_ID_NAV_PVT   0x07
#define UBX_NAV_PVT_LEN  92u
#define UBX_CLASS_ACK    0x05
#define UBX_ID_ACK_ACK   0x01
#define UBX_ID_ACK_NAK   0x00
#define UBX_CLASS_CFG    0x06

/* Bố cục payload UBX-NAV-PVT (little-endian, khớp Cortex-M4) */
typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon, lat, height, hMSL;
    uint32_t hAcc, vAcc;
    int32_t  velN, velE, velD, gSpeed, headMot;
    uint32_t sAcc, headAcc;
    uint16_t pDOP;
    uint8_t  reserved[6];   /* M8/M10: flags3 + reserved */
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} UBX_NavPvt_t;

_Static_assert(sizeof(UBX_NavPvt_t) == UBX_NAV_PVT_LEN, "UBX_NavPvt_t phai dai 92 byte");

typedef enum {
    S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB
} ParseState_t;

GPS_Data_t gps;
volatile GPS_Stats_t gps_stats;

static UART_HandleTypeDef *g_huart;
static uint8_t dma_buf[128];
static volatile uint8_t  ring[RING_SIZE];
static volatile uint16_t ring_head, ring_tail;

static ParseState_t st = S_SYNC1;
static uint8_t  msg_class, msg_id, ck_a, ck_b;
static uint16_t msg_len, msg_idx;
static uint8_t  payload[UBX_NAV_PVT_LEN + 8];

#define CK_UPDATE(b)  do { ck_a = (uint8_t)(ck_a + (b)); ck_b = (uint8_t)(ck_b + ck_a); } while(0)

static void decode_nav_pvt(void)
{
    UBX_NavPvt_t p;
    memcpy(&p, payload, sizeof(p));

    gps.iTOW    = p.iTOW;
    gps.year    = p.year;
    gps.month   = p.month;
    gps.day     = p.day;
    gps.hour    = p.hour;
    gps.min     = p.min;
    gps.sec     = p.sec;
    gps.fixType = p.fixType;
    gps.gnssFixOK = p.flags & 0x01;
    gps.carrSoln  = (p.flags >> 6) & 0x03;
    gps.numSV   = p.numSV;
    gps.lon     = p.lon;
    gps.lat     = p.lat;
    gps.height  = p.height;
    gps.hMSL    = p.hMSL;
    gps.hAcc    = p.hAcc;
    gps.vAcc    = p.vAcc;
    gps.velN    = p.velN;
    gps.velE    = p.velE;
    gps.velD    = p.velD;
    gps.gSpeed  = p.gSpeed;
    gps.headMot = p.headMot;
    gps.sAcc    = p.sAcc;
    gps.headAcc = p.headAcc;
    gps.pDOP    = p.pDOP;
    gps.rx_tick_ms = HAL_GetTick();
    if(gps.rx_tick_ms == 0) gps.rx_tick_ms = 1;   /* 0 dành cho "chưa có gói nào" */
}

/* Trả 1 khi vừa giải mã xong một gói NAV-PVT hợp lệ */
static int parse_byte(uint8_t b)
{
    switch (st) {
    case S_SYNC1:
        if (b == UBX_SYNC1) st = S_SYNC2;
        break;

    case S_SYNC2:
        if (b == UBX_SYNC2)      st = S_CLASS;
        else if (b == UBX_SYNC1) st = S_SYNC2;
        else                     st = S_SYNC1;
        break;

    case S_CLASS:
        msg_class = b; ck_a = 0; ck_b = 0; CK_UPDATE(b);
        st = S_ID;
        break;

    case S_ID:
        msg_id = b; CK_UPDATE(b);
        st = S_LEN1;
        break;

    case S_LEN1:
        msg_len = b; CK_UPDATE(b);
        st = S_LEN2;
        break;

    case S_LEN2:
        msg_len |= (uint16_t)b << 8; CK_UPDATE(b);
        msg_idx = 0;
        if (msg_len > sizeof(payload)) {
            st = S_SYNC1;            /* gói quá dài với buffer: bỏ qua */
        } else {
            st = (msg_len > 0) ? S_PAYLOAD : S_CKA;
        }
        break;

    case S_PAYLOAD:
        payload[msg_idx++] = b; CK_UPDATE(b);
        if (msg_idx >= msg_len) st = S_CKA;
        break;

    case S_CKA:
        if (b == ck_a) {
            st = S_CKB;
        } else {
            gps_stats.bad_ck++;
            st = S_SYNC1;
        }
        break;

    case S_CKB:
        st = S_SYNC1;
        if (b == ck_b) {
            if (msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_NAV_PVT && msg_len == UBX_NAV_PVT_LEN) {
                decode_nav_pvt();
                gps_stats.pvt_ok++;
                return 1;
            }
            if (msg_class == UBX_CLASS_ACK) {
                if (msg_id == UBX_ID_ACK_ACK) gps_stats.ack++;
                else if (msg_id == UBX_ID_ACK_NAK) gps_stats.nak++;
            }
        } else {
            gps_stats.bad_ck++;
        }
        break;
    }
    return 0;
}

/* ===================== CẤU HÌNH MODULE (gửi lệnh UBX) ===================== */
#if GPS_AUTOCONFIG

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    uint8_t frame[6 + 64 + 2];
    if (len > 64) return;
    frame[0] = UBX_SYNC1; frame[1] = UBX_SYNC2;
    frame[2] = cls;       frame[3] = id;
    frame[4] = (uint8_t)(len & 0xFF); frame[5] = (uint8_t)(len >> 8);
    if (len) memcpy(&frame[6], pl, len);
    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < 6u + len; i++) { a = (uint8_t)(a + frame[i]); b = (uint8_t)(b + a); }
    frame[6 + len] = a; frame[7 + len] = b;
    HAL_UART_Transmit(g_huart, frame, (uint16_t)(8u + len), 200);
}

/* Ghi số little-endian vào buffer, trả con trỏ sau khi ghi */
static uint8_t *put_le(uint8_t *p, uint32_t v, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) { *p++ = (uint8_t)(v & 0xFF); v >>= 8; }
    return p;
}

/* u-blox M9/M10: UBX-CFG-VALSET, chỉ ghi vào RAM (layer 0x01) */
static void cfg_m10(void)
{
    uint8_t pl[64], *p = pl;
    *p++ = 0x00; *p++ = 0x01; *p++ = 0x00; *p++ = 0x00;     /* version, layers=RAM, reserved */
    p = put_le(p, 0x10730001UL, 4); *p++ = 1;               /* CFG-UART1INPROT-UBX   = 1 */
    p = put_le(p, 0x10740001UL, 4); *p++ = 1;               /* CFG-UART1OUTPROT-UBX  = 1 */
    p = put_le(p, 0x10740002UL, 4); *p++ = 0;               /* CFG-UART1OUTPROT-NMEA = 0 */
    p = put_le(p, 0x20910007UL, 4); *p++ = 1;               /* CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 */
    p = put_le(p, 0x30210001UL, 4); p = put_le(p, GPS_RATE_MS, 2);  /* CFG-RATE-MEAS */
    p = put_le(p, 0x30210002UL, 4); p = put_le(p, 1, 2);            /* CFG-RATE-NAV = 1 */
    p = put_le(p, 0x40520001UL, 4); p = put_le(p, GPS_BAUD, 4);     /* CFG-UART1-BAUDRATE (đổi baud cuối cùng) */
    ubx_send(UBX_CLASS_CFG, 0x8A, pl, (uint16_t)(p - pl));
}

/* u-blox M8 trở về trước: CFG-MSG, CFG-RATE, cuối cùng CFG-PRT (đổi baud) */
static void cfg_m8(void)
{
    uint8_t pl[20], *p;

    const uint8_t msg[3] = { UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1 };   /* NAV-PVT mỗi chu kỳ trên cổng hiện tại */
    ubx_send(UBX_CLASS_CFG, 0x01, msg, 3);

    p = pl;
    p = put_le(p, GPS_RATE_MS, 2);  /* measRate ms */
    p = put_le(p, 1, 2);            /* navRate */
    p = put_le(p, 1, 2);            /* timeRef = GPS */
    ubx_send(UBX_CLASS_CFG, 0x08, pl, 6);

    p = pl;
    *p++ = 1;  *p++ = 0;            /* portID = UART1, reserved */
    p = put_le(p, 0, 2);            /* txReady */
    p = put_le(p, 0x000008D0UL, 4); /* mode 8N1 */
    p = put_le(p, GPS_BAUD, 4);     /* baudRate */
    p = put_le(p, 0x0001, 2);       /* inProtoMask: UBX */
    p = put_le(p, 0x0001, 2);       /* outProtoMask: chỉ UBX */
    p = put_le(p, 0, 2);            /* flags */
    p = put_le(p, 0, 2);            /* reserved */
    ubx_send(UBX_CLASS_CFG, 0x00, pl, 20);
}

static void set_mcu_baud(uint32_t baud)
{
    HAL_UART_AbortReceive(g_huart);
    g_huart->Init.BaudRate = baud;
    HAL_UART_Init(g_huart);   /* đã init rồi nên không gọi lại MspInit, chỉ đổi BRR */
}

static void gps_autoconfig(void)
{
    static const uint32_t bauds[] = { 9600UL, 38400UL, 57600UL, 115200UL };
    for (uint32_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        set_mcu_baud(bauds[i]);
        HAL_Delay(20);
        cfg_m10();      /* M10: đổi baud ngay sau lệnh này */
        HAL_Delay(20);
        cfg_m8();       /* M8: lệnh cuối đổi baud */
        HAL_Delay(50);
    }
    set_mcu_baud(GPS_BAUD);
    HAL_Delay(100);
    /* Gửi lại 1 lần ở baud mới để nhận ACK (xem gps_stats.ack trong Watch) */
    cfg_m10();
    cfg_m8();
}
#endif /* GPS_AUTOCONFIG */

static void rx_start(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(g_huart, dma_buf, sizeof(dma_buf)) == HAL_OK)
        __HAL_DMA_DISABLE_IT(g_huart->hdmarx, DMA_IT_HT);   /* chỉ cần IDLE/TC, tránh callback nửa buffer */
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    g_huart   = huart;
    ring_head = ring_tail = 0;
    st        = S_SYNC1;
    memset(&gps, 0, sizeof(gps));
#if GPS_AUTOCONFIG
    gps_autoconfig();
#endif
    __HAL_UART_CLEAR_OREFLAG(g_huart);
    rx_start();
}

int GPS_Poll(void)
{
    int got = 0;
    while (ring_tail != ring_head) {
        uint8_t b = ring[ring_tail];
        ring_tail = (uint16_t)((ring_tail + 1u) & RING_MASK);
        gps_stats.bytes++;
        if (parse_byte(b)) got = 1;
    }
    return got;
}

uint8_t GPS_IsAlive(void)
{
    return (gps.rx_tick_ms != 0) && ((HAL_GetTick() - gps.rx_tick_ms) < GPS_TIMEOUT_MS);
}

void GPS_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != g_huart) return;

    for (uint16_t i = 0; i < size; i++) {
        uint16_t next = (uint16_t)((ring_head + 1u) & RING_MASK);
        if (next == ring_tail) { gps_stats.overruns++; continue; }
        ring[ring_head] = dma_buf[i];
        ring_head = next;
    }
    rx_start();
}

void GPS_OnError(UART_HandleTypeDef *huart)
{
    if (huart != g_huart) return;
    gps_stats.uart_err++;
    rx_start();   /* HAL đã dừng nhận khi lỗi: bật lại */
}

#endif /* USE_GPS */
