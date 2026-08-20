#include "protocol_u7.h"
#include "protocol_frame.h"
#include "bsp_uart.h"
#include "usart.h"
#include <string.h>

extern UART_FIFO_t       u7_fifo;
extern UART_HandleTypeDef huart1;

static uint16_t u7_seq = 0x0001;

/*===== Session 状态机 =====*/
enum {
    U7_SESS_IDLE = 0,
    U7_SESS_SEND,
    U7_SESS_RETRY_DELAY,
    U7_SESS_WAIT_RESP,
    U7_SESS_DONE_OK,
    U7_SESS_DONE_ERR,
};

/* 帧接收子状态 (1B CMD: 帧头 7 字节) */
enum {
    S_HDR0, S_HDR1, S_CMD, S_SEQ_H, S_SEQ_L,
    S_LEN_H, S_LEN_L, S_DATA, S_CRC_L, S_CRC_H
};

typedef struct {
    uint8_t  state;
    uint8_t  cmd;                       /* 1 字节命令码 */
    uint8_t  tx_data[U7_MAX_DATA];
    uint16_t tx_len;
    uint8_t  rx_buf[U7_MAX_DATA + 2];
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
} U7_Session;

static U7_Session u7_sess;

/*===== 公开 API =====*/

void U7_Proto_Init(void)
{
    UART_ClearBuffer(&u7_fifo);
    u7_seq = 0x0001;
    memset(&u7_sess, 0, sizeof(u7_sess));
    u7_sess.state = U7_SESS_IDLE;
}

void U7_Proto_Run(void)
{
    if (u7_sess.state == U7_SESS_IDLE ||
        u7_sess.state == U7_SESS_DONE_OK ||
        u7_sess.state == U7_SESS_DONE_ERR) {
        return;
    }

    if (u7_sess.state == U7_SESS_RETRY_DELAY) {
        if (HAL_GetTick() < u7_sess.deadline) return;
        u7_sess.state = U7_SESS_SEND;
    }

    /*── SEND ──*/
    if (u7_sess.state == U7_SESS_SEND) {
        uint8_t  frame[FRAME_BUF_SIZE_1B];
        uint16_t frame_len;

        if (u7_sess.tx_len > 0)
            frame_len = Frame_Build_1B(frame, u7_sess.cmd, u7_seq,
                                       u7_sess.tx_data, u7_sess.tx_len);
        else
            frame_len = Frame_Build_1B(frame, u7_sess.cmd, u7_seq,
                                       NULL, 0);

        UART_ClearBuffer(&u7_fifo);
        UART_SendArray(&huart1, frame, frame_len);

        u7_sess.deadline    = HAL_GetTick() + u7_sess.timeout_ms;
        u7_sess.frame_state = S_HDR0;
        u7_sess.frame_pos   = 0;
        u7_sess.state       = U7_SESS_WAIT_RESP;
        return;
    }

    /*── WAIT_RESP ──*/
    if (u7_sess.state == U7_SESS_WAIT_RESP) {
        if (HAL_GetTick() > u7_sess.deadline) {
            if (++u7_sess.retry < u7_sess.max_retry) {
                u7_sess.deadline = HAL_GetTick() + U7_RETRY_DELAY;
                u7_sess.state = U7_SESS_RETRY_DELAY;
                return;
            }
            u7_sess.result = U7_PROTO_ERR_TIMEOUT;
            u7_sess.state  = U7_SESS_DONE_ERR;
            u7_seq = (u7_seq >= 0xFFFF) ? 0x0001 : (u7_seq + 1);
            return;
        }

        uint8_t byte;
        while (UART_ReadByte(&u7_fifo, &byte)) {
            uint8_t *fb = u7_sess.frame_buf;
            uint16_t *fp = &u7_sess.frame_pos;

            switch (u7_sess.frame_state) {
            case S_HDR0:
                if (byte == FRAME_HDR0) { fb[(*fp)++] = byte; u7_sess.frame_state = S_HDR1; }
                break;
            case S_HDR1:
                if (byte == FRAME_HDR1) { fb[(*fp)++] = byte; u7_sess.frame_state = S_CMD; }
                else { *fp = 0; u7_sess.frame_state = S_HDR0; }
                break;
            case S_CMD:    fb[(*fp)++] = byte; u7_sess.frame_state = S_SEQ_H; break;
            case S_SEQ_H:  fb[(*fp)++] = byte; u7_sess.frame_state = S_SEQ_L; break;
            case S_SEQ_L:  fb[(*fp)++] = byte; u7_sess.frame_state = S_LEN_H; break;
            case S_LEN_H:
                fb[(*fp)++] = byte;            /* 存 LEN_H 到 buf[5] */
                u7_sess.frame_state = S_LEN_L; /* 等 LEN_L */
                break;
            case S_LEN_L:
                fb[(*fp)++] = byte;            /* 存 LEN_L 到 buf[6] */
                /* 1B CMD: LEN = (buf[5]<<8) | buf[6] */
                u7_sess.frame_dlen = ((uint16_t)fb[5] << 8) | byte;
                if (u7_sess.frame_dlen > u7_sess.rx_max)
                    { *fp = 0; u7_sess.frame_state = S_HDR0; break; }
                u7_sess.frame_state = (u7_sess.frame_dlen > 0) ? S_DATA : S_CRC_L;
                break;
            case S_DATA:
                fb[(*fp)++] = byte;
                if (*fp >= 7 + u7_sess.frame_dlen)
                    u7_sess.frame_state = S_CRC_L;
                break;
            case S_CRC_L:
                fb[(*fp)++] = byte; u7_sess.frame_state = S_CRC_H; break;
            case S_CRC_H: {
                fb[(*fp)++] = byte;
                uint8_t  rx_cmd;
                uint16_t rx_len;
                if (Frame_Parse_1B(fb, *fp, &rx_cmd, NULL,
                                   u7_sess.rx_buf, &rx_len)) {
                    if (rx_cmd == u7_sess.cmd) {
                        u7_sess.rx_len = rx_len;
                        u7_sess.result = U7_PROTO_OK;
                        u7_sess.state  = U7_SESS_DONE_OK;
                    } else {
                        u7_sess.result = U7_PROTO_ERR_FRAME;
                        u7_sess.state  = U7_SESS_DONE_ERR;
                    }
                    u7_seq = (u7_seq >= 0xFFFF) ? 0x0001 : (u7_seq + 1);
                    return;
                }
                *fp = 0; u7_sess.frame_state = S_HDR0;
                break;
            }
            }
        }
    }
}

/*===== 便捷函数 (可重入) =====*/

static int U7_SendCmd_Start(uint8_t cmd, uint8_t *tx, uint16_t tx_len,
                             uint16_t rx_max, uint32_t timeout_ms)
{
    u7_sess.cmd        = cmd;
    u7_sess.tx_len     = tx_len;
    if (tx_len > 0) memcpy(u7_sess.tx_data, tx, tx_len);
    u7_sess.rx_max     = rx_max;
    u7_sess.timeout_ms = timeout_ms;
    u7_sess.max_retry  = U7_RETRY_MAX;
    u7_sess.retry      = 0;
    u7_sess.state      = U7_SESS_SEND;
    return U7_PROTO_PENDING;
}

static int U7_SendCmd_Result(void)
{
    u7_sess.state = U7_SESS_IDLE;
    if (u7_sess.rx_len >= 1 && u7_sess.rx_buf[0] == U7_STATUS_OK)
        return U7_PROTO_OK;
    return U7_PROTO_ERR_FRAME;
}

int U7_Ping(void)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: return U7_SendCmd_Start(U7_CMD_PING, NULL, 0, 1, 50);
    case U7_SESS_DONE_OK: return U7_SendCmd_Result();
    case U7_SESS_DONE_ERR: { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_GetVersion(uint8_t *major, uint8_t *minor)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: return U7_SendCmd_Start(U7_CMD_GET_VERSION, NULL, 0, 3, 50);
    case U7_SESS_DONE_OK:
        u7_sess.state = U7_SESS_IDLE;
        if (u7_sess.rx_len >= 3 && u7_sess.rx_buf[0] == U7_STATUS_OK) {
            if (major) *major = u7_sess.rx_buf[1];
            if (minor) *minor = u7_sess.rx_buf[2];
            return U7_PROTO_OK;
        }
        return U7_PROTO_ERR_FRAME;
    case U7_SESS_DONE_ERR:
        { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_MotorStep(uint8_t motor, uint8_t direction, uint16_t steps)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: {
        uint8_t tx[4] = { motor, direction, (uint8_t)(steps >> 8), (uint8_t)steps };
        return U7_SendCmd_Start(U7_CMD_MOTOR_STEP, tx, 4, 1, 5000);
    }
    case U7_SESS_DONE_OK: return U7_SendCmd_Result();
    case U7_SESS_DONE_ERR: { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_MotorStop(uint8_t motor)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: {
        uint8_t tx[1] = { motor };
        return U7_SendCmd_Start(U7_CMD_MOTOR_STOP, tx, 1, 1, 100);
    }
    case U7_SESS_DONE_OK: return U7_SendCmd_Result();
    case U7_SESS_DONE_ERR: { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_ReadInput(uint8_t index, uint8_t *level)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: {
        uint8_t tx[1] = { index };
        return U7_SendCmd_Start(U7_CMD_READ_INPUT, tx, 1, 2, 50);
    }
    case U7_SESS_DONE_OK:
        u7_sess.state = U7_SESS_IDLE;
        if (u7_sess.rx_len >= 2 && u7_sess.rx_buf[0] == U7_STATUS_OK) {
            if (level) *level = u7_sess.rx_buf[1];
            return U7_PROTO_OK;
        }
        return U7_PROTO_ERR_FRAME;
    case U7_SESS_DONE_ERR:
        { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_SelfTest(uint8_t *result)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: return U7_SendCmd_Start(U7_CMD_SELF_TEST, NULL, 0, 2, 1000);
    case U7_SESS_DONE_OK:
        u7_sess.state = U7_SESS_IDLE;
        if (u7_sess.rx_len >= 2 && u7_sess.rx_buf[0] == U7_STATUS_OK) {
            if (result) *result = u7_sess.rx_buf[1];
            return U7_PROTO_OK;
        }
        return U7_PROTO_ERR_FRAME;
    case U7_SESS_DONE_ERR:
        { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}

int U7_Reset(void)
{
    if (u7_sess.state != U7_SESS_IDLE &&
        u7_sess.state != U7_SESS_DONE_OK &&
        u7_sess.state != U7_SESS_DONE_ERR) {
        return U7_PROTO_ERR_BUSY;
    }
    uint8_t  frame[FRAME_BUF_SIZE_1B];
    uint16_t seq   = u7_seq;
    uint16_t f_len = Frame_Build_1B(frame, U7_CMD_RESET, seq, NULL, 0);
    UART_ClearBuffer(&u7_fifo);
    UART_SendArray(&huart1, frame, f_len);
    u7_seq = (seq >= 0xFFFF) ? 0x0001 : (seq + 1);
    u7_sess.state = U7_SESS_IDLE;
    return U7_PROTO_OK;
}

int U7_GetADC(uint8_t *buf)
{
    switch (u7_sess.state) {
    case U7_SESS_IDLE: return U7_SendCmd_Start(U7_CMD_GET_ADC, NULL, 0, 16, 500);
    case U7_SESS_DONE_OK:
        u7_sess.state = U7_SESS_IDLE;
        if (u7_sess.rx_len >= 16) {
            if (buf) memcpy(buf, u7_sess.rx_buf, 16);
            return U7_PROTO_OK;
        }
        return U7_PROTO_ERR_FRAME;
    case U7_SESS_DONE_ERR:
        { int r = u7_sess.result; u7_sess.state = U7_SESS_IDLE; return r; }
    default: return U7_PROTO_PENDING;
    }
}
