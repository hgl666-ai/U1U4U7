#include "protocol_u4.h"
#include "protocol_frame.h"
#include "bsp_uart.h"
#include "usart.h"
#include <string.h>

extern UART_FIFO_t       u4_fifo;
extern UART_HandleTypeDef huart2;

static uint16_t u4_seq = 0x0001;

/*===== 报文 =====*/
U4_ReportData u4_report;
uint8_t       u4_report_fresh = 0;
/* [2026-08-20] U4 报文最后到达时刻 (ms): 用于 U4 在线检测,
 * 校准/一致性命令启动时 U4 离线 → 立即返回错误, 不空跑/不喂脏数据 */
volatile uint32_t u4_report_tick = 0;

/* ISP 烧录期间暂停扫描: U4_ScanReport 会吃掉 ISP 的 0x79 响应 */
uint8_t u4_scan_pause = 0;

void U4_ScanPause(uint8_t pause)
{
    u4_scan_pause = pause;
}

/**
  * @brief  解析 24 字节 ccd_data_t 报文 (大端序)
  */
static void U4_ParseReport(uint8_t *data, uint16_t len)
{
    if (len < 24) return;
    u4_report.length    = (int16_t)(((uint16_t)data[0]  << 8) | data[1]);
    u4_report.raw_length = (int16_t)(((uint16_t)data[2]  << 8) | data[3]);
    u4_report.position   = (int16_t)(((uint16_t)data[4]  << 8) | data[5]);
    u4_report.vo1        = ((uint16_t)data[6]  << 8) | data[7];
    u4_report.vo2        = ((uint16_t)data[8]  << 8) | data[9];
    u4_report.min_vo1    = ((uint16_t)data[10] << 8) | data[11];
    u4_report.max_vo1    = ((uint16_t)data[12] << 8) | data[13];
    u4_report.min_vo2    = ((uint16_t)data[14] << 8) | data[15];
    u4_report.max_vo2    = ((uint16_t)data[16] << 8) | data[17];
    u4_report.angle_acc  = (int32_t)(((uint32_t)data[18] << 24)
                           | ((uint32_t)data[19] << 16)
                           | ((uint32_t)data[20] << 8)
                           |  (uint32_t)data[21]);
    u4_report.ams_adc    = ((uint16_t)data[22] << 8) | data[23];
    u4_report_fresh = 1;
    u4_report_tick  = HAL_GetTick();
}

/*===== U4 在线检测 [2026-08-20] =====
 * U4 报文流 ~33B/50ms, 超过 U4_OFFLINE_TIMEOUT_MS 未收到报文即视为离线 */
uint8_t U4_IsOnline(void)
{
    return (HAL_GetTick() - u4_report_tick) < U4_OFFLINE_TIMEOUT_MS;
}

/*===== Session 状态机 =====*/
enum {
    U4_SESS_IDLE = 0,
    U4_SESS_SEND,
    U4_SESS_RETRY_DELAY,
    U4_SESS_WAIT_RESP,
    U4_SESS_DONE_OK,
    U4_SESS_DONE_ERR,
};

/* 帧接收子状态 (1B CMD) 和空闲报文扫描共用 */
enum {
    S_HDR0, S_HDR1, S_CMD, S_SEQ_H, S_SEQ_L,
    S_LEN_H, S_LEN_L, S_DATA, S_CRC_L, S_CRC_H
};

typedef struct {
    uint8_t  state;
    uint8_t  cmd;
    /* [2026-08-17] 内存优化 (RAM 10KB 溢出修复): tx_data 只在 SEND 状态使用,
     * rx_buf 只在 WAIT_RESP/DONE 状态使用, 生命周期互斥 → 共用一块, 省 256B */
    union {
        uint8_t tx_data[U4_MAX_DATA];
        uint8_t rx_buf[U4_MAX_DATA + 2];
    } io;
    uint16_t tx_len;
    uint16_t rx_max;
    uint16_t rx_len;
    int      result;
    uint32_t deadline;
    uint8_t  retry;
    uint8_t  max_retry;
    uint32_t timeout_ms;

    uint8_t  frame_state;
    uint8_t  frame_buf[FRAME_BUF_SIZE_1B];
    uint16_t frame_pos;
    uint16_t frame_dlen;
} U4_Session;

static U4_Session u4_sess;

/* SEND 帧构建 与 报文解析 共用一块缓冲 (状态互斥: SEND/IDLE), 减 .bss [2026-08-17] */
static uint8_t u4_tmp[FRAME_BUF_SIZE_1B];

/* 空闲时收报文的帧状态: 帧缓冲复用 u4_sess.frame_buf (仅 IDLE 状态下使用, 会话缓冲空闲) */
static uint8_t  idle_frame_state = S_HDR0;
static uint16_t idle_frame_pos   = 0;
static uint16_t idle_frame_dlen  = 0;

/**
  * @brief  空闲时扫描 FIFO 中的 0x30 报文帧
  */
static void U4_ScanReport(void)
{
    if (u4_scan_pause) return;   /* ISP 期间暂停, 避免吃掉 ISP 响应 */
    uint8_t byte;
    while (UART_ReadByte(&u4_fifo, &byte)) {
        uint8_t *fb = u4_sess.frame_buf;   /* 复用会话帧缓冲 (IDLE 状态空闲) */
        uint16_t *fp = &idle_frame_pos;

        switch (idle_frame_state) {
        case S_HDR0:
            if (byte == FRAME_HDR0) { fb[(*fp)++] = byte; idle_frame_state = S_HDR1; }
            break;
        case S_HDR1:
            if (byte == FRAME_HDR1) { fb[(*fp)++] = byte; idle_frame_state = S_CMD; }
            else { *fp = 0; idle_frame_state = S_HDR0; }
            break;
        case S_CMD:    fb[(*fp)++] = byte; idle_frame_state = S_SEQ_H; break;
        case S_SEQ_H:  fb[(*fp)++] = byte; idle_frame_state = S_SEQ_L; break;
        case S_SEQ_L:  fb[(*fp)++] = byte; idle_frame_state = S_LEN_H; break;
        case S_LEN_H:
            fb[(*fp)++] = byte;            /* 存 LEN_H */
            idle_frame_state = S_LEN_L;     /* 等 LEN_L */
            break;
        case S_LEN_L:
            fb[(*fp)++] = byte;            /* 存 LEN_L */
            idle_frame_dlen = ((uint16_t)fb[5] << 8) | byte;
            if (idle_frame_dlen > U4_MAX_DATA) { *fp = 0; idle_frame_state = S_HDR0; break; }
            idle_frame_state = (idle_frame_dlen > 0) ? S_DATA : S_CRC_L;
            break;
        case S_DATA:
            fb[(*fp)++] = byte;
            if (*fp >= 7 + idle_frame_dlen) idle_frame_state = S_CRC_L;
            break;
        case S_CRC_L:
            fb[(*fp)++] = byte; idle_frame_state = S_CRC_H; break;
        case S_CRC_H: {
            fb[(*fp)++] = byte;
            uint8_t  rx_cmd;
            uint16_t dlen;
            if (Frame_Parse_1B_LE(fb, *fp, &rx_cmd, NULL, u4_tmp, &dlen)) {
                if (rx_cmd == U4_CMD_REPORT) {
                    U4_ParseReport(u4_tmp, dlen);
                }
                /* 非 0x30 的帧丢弃 */
            }
            *fp = 0; idle_frame_state = S_HDR0;
            break;
        }
        }
    }
}

/*===== 公开 API =====*/

void U4_Proto_Init(void)
{
    UART_ClearBuffer(&u4_fifo);
    u4_seq = 0x0001;
    memset(&u4_sess, 0, sizeof(u4_sess));
    u4_sess.state = U4_SESS_IDLE;
    memset(&u4_report, 0, sizeof(u4_report));
    u4_report_fresh = 0;
    idle_frame_state = S_HDR0;
    idle_frame_pos   = 0;
}

void U4_Proto_Run(void)
{
    /* DMA 看门狗: 若 USART2 DMA 接收已停止 (帧错误触发, 问题3), 自动重启
     * 只在非 ISP 暂停期生效; 烧录完成后 ISP_Go 已恢复 DMA, 若之后又停则这里拉起 */
    if (!u4_scan_pause && huart2.hdmarx &&
        (huart2.hdmarx->Instance->CCR & DMA_CCR_EN) == 0) {
        extern uint8_t u4_dma_buf[];
        extern UART_FIFO_t u4_fifo;
        __HAL_UART_CLEAR_FLAG(&huart2, USART_SR_ORE | USART_SR_FE | USART_SR_NE);
        HAL_UART_Receive_DMA(&huart2, u4_dma_buf, u4_fifo.dma_ndtr_init);
        if (huart2.hdmarx) u4_fifo.last_dma_count = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    }

    /*── 空闲时扫描报文 ──*/
    if (u4_sess.state == U4_SESS_IDLE) {
        U4_ScanReport();
        return;
    }

    if (u4_sess.state == U4_SESS_DONE_OK ||
        u4_sess.state == U4_SESS_DONE_ERR) {
        return;
    }

    if (u4_sess.state == U4_SESS_RETRY_DELAY) {
        if (HAL_GetTick() < u4_sess.deadline) return;
        u4_sess.state = U4_SESS_SEND;
    }

    /*── SEND ──*/
    if (u4_sess.state == U4_SESS_SEND) {
        /* 帧构建复用 u4_tmp (与 IDLE 报文解析互斥) */
        uint16_t frame_len;

        if (u4_sess.tx_len > 0)
            frame_len = Frame_Build_1B_LE(u4_tmp, u4_sess.cmd, u4_seq,
                                       u4_sess.io.tx_data, u4_sess.tx_len);
        else
            frame_len = Frame_Build_1B_LE(u4_tmp, u4_sess.cmd, u4_seq,
                                       NULL, 0);

        UART_ClearBuffer(&u4_fifo);
        UART_SendArray(&huart2, u4_tmp, frame_len);

        u4_sess.deadline    = HAL_GetTick() + u4_sess.timeout_ms;
        u4_sess.frame_state = S_HDR0;
        u4_sess.frame_pos   = 0;
        u4_sess.state       = U4_SESS_WAIT_RESP;
        return;
    }

    /*── WAIT_RESP ──*/
    if (u4_sess.state == U4_SESS_WAIT_RESP) {
        if (HAL_GetTick() > u4_sess.deadline) {
            if (++u4_sess.retry < u4_sess.max_retry) {
                u4_sess.deadline = HAL_GetTick() + U4_RETRY_DELAY;
                u4_sess.state = U4_SESS_RETRY_DELAY;
                return;
            }
            u4_sess.result = U4_PROTO_ERR_TIMEOUT;
            u4_sess.state  = U4_SESS_DONE_ERR;
            u4_seq = (u4_seq >= 0xFFFF) ? 0x0001 : (u4_seq + 1);
            return;
        }

        uint8_t byte;
        while (UART_ReadByte(&u4_fifo, &byte)) {
            uint8_t *fb = u4_sess.frame_buf;
            uint16_t *fp = &u4_sess.frame_pos;

            switch (u4_sess.frame_state) {
            case S_HDR0:
                if (byte == FRAME_HDR0) { fb[(*fp)++] = byte; u4_sess.frame_state = S_HDR1; }
                break;
            case S_HDR1:
                if (byte == FRAME_HDR1) { fb[(*fp)++] = byte; u4_sess.frame_state = S_CMD; }
                else { *fp = 0; u4_sess.frame_state = S_HDR0; }
                break;
            case S_CMD:    fb[(*fp)++] = byte; u4_sess.frame_state = S_SEQ_H; break;
            case S_SEQ_H:  fb[(*fp)++] = byte; u4_sess.frame_state = S_SEQ_L; break;
            case S_SEQ_L:  fb[(*fp)++] = byte; u4_sess.frame_state = S_LEN_H; break;
            case S_LEN_H:
                fb[(*fp)++] = byte;            /* 存 LEN_H */
                u4_sess.frame_state = S_LEN_L; /* 等 LEN_L */
                break;
            case S_LEN_L:
                fb[(*fp)++] = byte;            /* 存 LEN_L */
                u4_sess.frame_dlen = ((uint16_t)fb[5] << 8) | byte;
                if (u4_sess.frame_dlen > u4_sess.rx_max)
                    { *fp = 0; u4_sess.frame_state = S_HDR0; break; }
                u4_sess.frame_state = (u4_sess.frame_dlen > 0) ? S_DATA : S_CRC_L;
                break;
            case S_DATA:
                fb[(*fp)++] = byte;
                if (*fp >= 7 + u4_sess.frame_dlen)
                    u4_sess.frame_state = S_CRC_L;
                break;
            case S_CRC_L:
                fb[(*fp)++] = byte; u4_sess.frame_state = S_CRC_H; break;
            case S_CRC_H: {
                fb[(*fp)++] = byte;
                uint8_t  rx_cmd;
                uint16_t rx_len;
                if (Frame_Parse_1B_LE(fb, *fp, &rx_cmd, NULL,
                                   u4_sess.io.rx_buf, &rx_len)) {
                    if (rx_cmd == u4_sess.cmd) {
                        u4_sess.rx_len = rx_len;
                        u4_sess.result = U4_PROTO_OK;
                        u4_sess.state  = U4_SESS_DONE_OK;
                    } else if (rx_cmd == U4_CMD_REPORT) {
                        /* 等待过程中收到报文: 解析并继续等 */
                        U4_ParseReport(u4_sess.io.rx_buf, rx_len);
                        *fp = 0; u4_sess.frame_state = S_HDR0;
                        break;  /* 不退出，继续收 */
                    } else {
                        u4_sess.result = U4_PROTO_ERR_FRAME;
                        u4_sess.state  = U4_SESS_DONE_ERR;
                    }
                    if (u4_sess.state != U4_SESS_WAIT_RESP) {
                        u4_seq = (u4_seq >= 0xFFFF) ? 0x0001 : (u4_seq + 1);
                        return;
                    }
                    break;  /* 报文已处理，继续循环 */
                }
                *fp = 0; u4_sess.frame_state = S_HDR0;
                break;
            }
            }
        }
    }
}

/*===== 便捷函数 =====
 * 注意: 全部便捷函数共享单一 u4_sess, 非可重入!
 * 同一时刻只允许一个命令发起方; 他人调用会消费掉当前会话的结果 (详见审查报告 M1) */

static int U4_SendCmd_Start(uint8_t cmd, uint8_t *tx, uint16_t tx_len,
                             uint16_t rx_max, uint32_t timeout_ms)
{
    u4_sess.cmd        = cmd;
    u4_sess.tx_len     = tx_len;
    if (tx_len > 0) memcpy(u4_sess.io.tx_data, tx, tx_len);
    u4_sess.rx_max     = rx_max;
    u4_sess.timeout_ms = timeout_ms;
    u4_sess.max_retry  = U4_RETRY_MAX;
    u4_sess.retry      = 0;
    u4_sess.state      = U4_SESS_SEND;
    return U4_PROTO_PENDING;
}

static int U4_SendCmd_Result(void)
{
    u4_sess.state = U4_SESS_IDLE;
    /* [2026-08-17] M4 修复: 空 ACK (rx_len==0) 仍为合法成功;
     * 带 1 字节状态时: 0=OK, 非0=设备错误码。
     * (此前这类命令 rx_max=0, 设备回错误状态帧会在 LEN 检查处被丢弃, 误报 TIMEOUT) */
    if (u4_sess.rx_len == 0)
        return U4_PROTO_OK;
    if (u4_sess.io.rx_buf[0] == U4_STATUS_OK)
        return U4_PROTO_OK;
    return U4_PROTO_ERR_DEVICE;
}

int U4_ZeroSensor(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_ZERO_SENSOR, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_StartCalib(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_START_CALIB, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_FinishCalib(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_FINISH_CALIB, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_CancelCalib(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_CANCEL_CALIB, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_ReadFlashParam(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_READ_FLASH_PARAM, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_SaveFlashParam(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_SAVE_FLASH_PARAM, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_FactoryReset(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_FACTORY_RESET, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_SetOffset(uint16_t offset_um)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: {
        uint8_t tx[2] = { (uint8_t)(offset_um >> 8), (uint8_t)(offset_um) };
        /* [2026-08-18] 带数据命令禁重试: io.tx_data 与 io.rx_buf 共用内存,
         * 重试时 tx 可能已被收到的响应字节覆盖 → 重发帧内容错误 */
        int r = U4_SendCmd_Start(U4_CMD_SET_OFFSET, tx, 2, 1, 200);
        u4_sess.max_retry = 1;
        return r;
    }
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_SetReportPeriod(uint16_t period_ms)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: {
        uint8_t tx[2] = { (uint8_t)(period_ms >> 8), (uint8_t)(period_ms) };
        /* [2026-08-18] 同上: 带数据命令禁重试 */
        int r = U4_SendCmd_Start(U4_CMD_SET_REPORT_PERIOD, tx, 2, 1, 200);
        u4_sess.max_retry = 1;
        return r;
    }
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_AmsZero(void)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_AMS_ZERO, NULL, 0, 1, 200);
    case U4_SESS_DONE_OK: return U4_SendCmd_Result();
    case U4_SESS_DONE_ERR: { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

/*===== 兼容旧命令 (自拟, 待 U4 方确认) =====*/

int U4_GetVersion(uint8_t *major, uint8_t *minor, uint8_t *revision)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_GET_VERSION, NULL, 0, 3, 50);
    case U4_SESS_DONE_OK:
        u4_sess.state = U4_SESS_IDLE;
        if (u4_sess.rx_len >= 3) {
            if (major)    *major    = u4_sess.io.rx_buf[0];
            if (minor)    *minor    = u4_sess.io.rx_buf[1];
            if (revision) *revision = u4_sess.io.rx_buf[2];
            return U4_PROTO_OK;
        }
        return U4_PROTO_ERR_FRAME;
    case U4_SESS_DONE_ERR:
        { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}

int U4_ReadAllADC(uint8_t *buf)
{
    switch (u4_sess.state) {
    case U4_SESS_IDLE: return U4_SendCmd_Start(U4_CMD_READ_ALL_ADC, NULL, 0, 16, 200);
    case U4_SESS_DONE_OK:
        u4_sess.state = U4_SESS_IDLE;
        if (u4_sess.rx_len >= 16) {
            if (buf) memcpy(buf, u4_sess.io.rx_buf, 16);
            return U4_PROTO_OK;
        }
        return U4_PROTO_ERR_FRAME;
    case U4_SESS_DONE_ERR:
        { int r = u4_sess.result; u4_sess.state = U4_SESS_IDLE; return r; }
    default: return U4_PROTO_PENDING;
    }
}
