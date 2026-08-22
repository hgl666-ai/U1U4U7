#include "protocol.h"
#include "protocol_frame.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_motor.h"
#include "bsp_tmc2209.h"   /* U7_DEBUG 总开关定义处 */
#include <string.h>

extern UART_FIFO_t uart1_fifo;

#if U7_DEBUG
/* 调试打印 (直接写 USART1->DR, 仅在定义 U7_DEBUG 时启用, 不污染正常协议帧) */
static void motor_dbg_print(const char *s)
{
    while (*s) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = (uint8_t)(*s++);
    }
}
static void motor_dbg_print_u16(uint16_t v)
{
    char buf[6];
    int8_t i = 5;
    buf[i] = 0;
    do { buf[--i] = (char)('0' + v % 10); v /= 10; } while (v > 0);
    motor_dbg_print(&buf[i]);
}
#endif /* U7_DEBUG */

/*===== 收帧状态机 (1B CMD: A5 5B + 1B CMD + 2B SEQ + 2B LEN + DATA + 2B CRC16) =====*/
enum {
    S_HDR0, S_HDR1, S_CMD, S_SEQ_H, S_SEQ_L,
    S_LEN_H, S_LEN_L, S_DATA, S_CRC_L, S_CRC_H
};

static uint8_t  rx_state = S_HDR0;
static uint8_t  rx_buf[FRAME_BUF_SIZE_1B];
static uint16_t rx_pos  = 0;
static uint16_t rx_dlen = 0;
static uint16_t req_seq = 0;

/* 收帧数据 与 响应构建 共用一块缓冲 (解析完成后 handler 用其构造响应, 减 .bss) [2026-08-17] */
static uint8_t u7_tmp[FRAME_BUF_SIZE_1B];

/*===== 发送响应 =====*/

static void SendResponse(uint8_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t total = Frame_Build_1B(u7_tmp, cmd, req_seq, data, len);
    UART_SendArray(&huart1, u7_tmp, total);
}

/*===== 命令处理 =====*/

static void HandlePing(void)
{
#if U7_DEBUG
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = 'P';  /* PING被调, COM4可见 */
#endif
    uint8_t resp = PROTO_STATUS_OK;
    SendResponse(PROTO_CMD_PING, &resp, 1);
}

static void HandleGetVersion(void)
{
    uint8_t resp[3] = { PROTO_STATUS_OK, PROTO_FW_MAJOR, PROTO_FW_MINOR };
    SendResponse(PROTO_CMD_GET_VERSION, resp, 3);
}

static void HandleMotorStep(uint8_t *data, uint16_t len)
{
    if (len < 4) {
#if U7_DEBUG
        motor_dbg_print("STEP reject: len<4\r\n");
#endif
        uint8_t e = PROTO_STATUS_ERROR;
        SendResponse(PROTO_CMD_MOTOR_STEP, &e, 1); return;
    }
    uint8_t  motor = data[0];
    uint8_t  dir   = data[1];
    uint16_t steps = ((uint16_t)data[2] << 8) | data[3];
    if (motor != MOTOR_ID || steps == 0) {
#if U7_DEBUG
        motor_dbg_print("STEP reject: motor/step=0\r\n");
#endif
        uint8_t e = PROTO_STATUS_ERROR;
        SendResponse(PROTO_CMD_MOTOR_STEP, &e, 1); return;
    }
    /* [2026-08-17] M7 修复: 电机运动中拒绝新指令时必须回错误,
     * 此前忽略指令却回 OK, U1 会误以为已执行 (开环定位累积误差) */
    if (!BSP_Motor_Move(dir, steps, MOTOR_SPEED_SLOW)) {
#if U7_DEBUG
        motor_dbg_print("STEP reject: MOVE=0 (原因见 bsp_motor MOVE reject)\r\n");
#endif
        uint8_t e = PROTO_STATUS_ERROR;
        SendResponse(PROTO_CMD_MOTOR_STEP, &e, 1); return;
    }
#if U7_DEBUG
    motor_dbg_print("STEP ok, steps=");
    motor_dbg_print_u16(steps);
    motor_dbg_print("\r\n");
#endif
    uint8_t ok = PROTO_STATUS_OK;
    SendResponse(PROTO_CMD_MOTOR_STEP, &ok, 1);
}

static void HandleMotorStop(uint8_t *data, uint16_t len)
{
    if (len < 1 || data[0] != MOTOR_ID) {
        uint8_t e = PROTO_STATUS_ERROR;
        SendResponse(PROTO_CMD_MOTOR_STOP, &e, 1); return;
    }
    BSP_Motor_Stop();
    uint8_t ok = PROTO_STATUS_OK;
    SendResponse(PROTO_CMD_MOTOR_STOP, &ok, 1);
}

/* [2026-08-20] 电机回零: 反转(CCW)低速找 IN1(PB9) 零点
 * 阻塞等待回零完成 (最坏 3200 步@500Hz=6.4s, 兜底 8s), 完成后回 OK/ERROR
 * [2026-08-21] 兜底 4s→8s: MAX_STEPS 加大到 3200 后, 4s 会在电机走完前强停 */
static void HandleHome(void)
{
    if (BSP_Motor_IsBusy()) {
        uint8_t e = PROTO_STATUS_ERROR;
        SendResponse(PROTO_CMD_HOME, &e, 1); return;
    }
#if U7_DEBUG
    /* [2026-08-21] 回零前 IN 电平: 确认未触发时读 1 (外部上拉) */
    motor_dbg_print("HOME start IN1=");
    motor_dbg_print_u16(BSP_IN_Read(1));
    motor_dbg_print(" IN2=");
    motor_dbg_print_u16(BSP_IN_Read(2));
    motor_dbg_print("\r\n");
#endif
    BSP_Motor_Home();
    uint32_t t0 = HAL_GetTick();
    while (!BSP_Motor_HomeDone() && !BSP_Motor_HomeFail()) {
        if (HAL_GetTick() - t0 > 8000) break;   /* 兜底超时 (3200步@500Hz=6.4s + 余量) */
    }
    /* [2026-08-20] 兜底超时后强制停机: 若电机仍在转 (busy 残留=1),
     * 后续所有 MotorStep/HOME 会被拒 → "成功一次后电机不动但有力" */
    if (BSP_Motor_IsBusy()) BSP_Motor_Stop();
#if U7_DEBUG
    /* [2026-08-21] 状态打印移到主循环 (ISR 内打印阻塞 USART ~3ms 会丢脉冲);
     * 回零后 IN 电平: 若电机压到开关但 IN1 仍=1 → 触发极性反/开关未接线;
     * 若 IN1=0 → 开关触发正常但 ISR 未捕获 (去抖/时序问题) */
    if (BSP_Motor_HomeDone())             motor_dbg_print("HOME done (IN1)\r\n");
    else if (BSP_Motor_HomeFail())        motor_dbg_print("HOME fail (timeout/IN2)\r\n");
    else                                  motor_dbg_print("HOME fail (4s watchdog)\r\n");
    motor_dbg_print("HOME end  IN1=");
    motor_dbg_print_u16(BSP_IN_Read(1));
    motor_dbg_print(" IN2=");
    motor_dbg_print_u16(BSP_IN_Read(2));
    motor_dbg_print("\r\n");
#endif
    uint8_t ok = (BSP_Motor_HomeDone() && !BSP_Motor_HomeFail())
                 ? PROTO_STATUS_OK : PROTO_STATUS_ERROR;
    SendResponse(PROTO_CMD_HOME, &ok, 1);
}

static void HandleReadInput(uint8_t *data, uint16_t len)
{
    if (len < 1) {
        uint8_t resp[2] = { PROTO_STATUS_ERROR, 0 };
        SendResponse(PROTO_CMD_READ_INPUT, resp, 2); return;
    }
    uint8_t level = BSP_IN_Read(data[0]);
    uint8_t resp[2] = { PROTO_STATUS_OK, level };
    SendResponse(PROTO_CMD_READ_INPUT, resp, 2);
}

static void HandleSelfTest(void)
{
    uint8_t result = 0;
    ADC_Value_t v0 = BSP_ADC_ReadChannel(ADC_CH_ADC0, 8);
    ADC_Value_t v1 = BSP_ADC_ReadChannel(ADC_CH_ADC1, 8);
    if (!v0.valid || !v1.valid) result = 1;
    BSP_IN_Read(1); BSP_IN_Read(2);
    uint8_t resp[2] = { (uint8_t)(result ? PROTO_STATUS_ERROR : PROTO_STATUS_OK), result };
    SendResponse(PROTO_CMD_SELF_TEST, resp, 2);
}

static void HandleGetADC(void)
{
#if U7_DEBUG
    USART1->DR = 'G'; while(!(USART1->SR&USART_SR_TC)); /* 看到G说明Handler被调了 */
#endif
    static const uint32_t ch[4] = {ADC_CH_ADC0, ADC_CH_ADC1, ADC_CH_ADC4, ADC_CH_ADC5};
    static const uint8_t  id[4] = {1, 2, 3, 4};
    uint8_t buf[16];
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t ok;
        uint16_t raw = BSP_ADC_ReadRaw(ch[i], &ok);
        buf[i * 4 + 0] = id[i];
        buf[i * 4 + 1] = (uint8_t)(raw >> 8);
        buf[i * 4 + 2] = (uint8_t)(raw & 0xFF);
        buf[i * 4 + 3] = ok ? 1 : 0;
    }
    SendResponse(PROTO_CMD_GET_ADC, buf, 16);
}

static void HandleReset(void)
{
    NVIC_SystemReset();
}

static void Dispatch(uint8_t cmd, uint8_t *data, uint16_t len)
{
    switch (cmd) {
        case PROTO_CMD_PING:         HandlePing();                break;
        case PROTO_CMD_GET_VERSION:  HandleGetVersion();          break;
        case PROTO_CMD_MOTOR_STEP:   HandleMotorStep(data, len);  break;
        case PROTO_CMD_MOTOR_STOP:   HandleMotorStop(data, len);  break;
        case PROTO_CMD_READ_INPUT:   HandleReadInput(data, len);  break;
        case PROTO_CMD_SELF_TEST:    HandleSelfTest();            break;
        case PROTO_CMD_GET_ADC:      HandleGetADC();              break;
        case PROTO_CMD_HOME:         HandleHome();                break;
        case PROTO_CMD_RESET:        HandleReset();               break;
        default: {
            /* 通用回声: 收到任何帧都回复 PING ACK 证明收到 */
            uint8_t ok = PROTO_STATUS_OK;
            SendResponse(PROTO_CMD_PING, &ok, 1); break;
        }
    }
}

/*===== 主循环 =====*/

void PROTO_Init(void)
{
    rx_state = S_HDR0;
    rx_pos   = 0;
}

void PROTO_Run(void)
{
    uint8_t byte;
    while (UART_ReadByte(&uart1_fifo, &byte)) {
        switch (rx_state) {
        case S_HDR0:
            if (byte == FRAME_HDR0) { rx_buf[rx_pos++] = byte; rx_state = S_HDR1; }
            break;
        case S_HDR1:
            if (byte == FRAME_HDR1) { rx_buf[rx_pos++] = byte; rx_state = S_CMD; }
            else { rx_pos = 0; rx_state = S_HDR0; }
            break;
        case S_CMD:    rx_buf[rx_pos++] = byte; rx_state = S_SEQ_H; break;
        case S_SEQ_H:  rx_buf[rx_pos++] = byte; rx_state = S_SEQ_L; break;
        case S_SEQ_L:  rx_buf[rx_pos++] = byte; rx_state = S_LEN_H; break;
        case S_LEN_H:
            rx_buf[rx_pos++] = byte;       /* 存 LEN_H 到 buf[5] */
            rx_state = S_LEN_L;            /* 等 LEN_L */
            break;
        case S_LEN_L:
            rx_buf[rx_pos++] = byte;       /* 存 LEN_L 到 buf[6] */
            /* 1B CMD: LEN = (buf[5] << 8) | buf[6] */
            rx_dlen = ((uint16_t)rx_buf[5] << 8) | byte;
            if (rx_dlen > PROTO_MAX_DATA) { rx_pos = 0; rx_state = S_HDR0; break; }
            rx_state = (rx_dlen > 0) ? S_DATA : S_CRC_L;
            break;
        case S_DATA:
            rx_buf[rx_pos++] = byte;
            if (rx_pos >= 7 + rx_dlen) rx_state = S_CRC_L;
            break;
        case S_CRC_L:
            rx_buf[rx_pos++] = byte; rx_state = S_CRC_H; break;
        case S_CRC_H:
            rx_buf[rx_pos++] = byte;
            {
                uint8_t  cmd; uint16_t seq;
                uint16_t len;
                if (Frame_Parse_1B(rx_buf, rx_pos, &cmd, &seq, u7_tmp, &len)) {
                    req_seq = seq;
#if U7_DEBUG
                    while (!(USART1->SR & USART_SR_TXE));
                    USART1->DR = 'P';
                    while (!(USART1->SR & USART_SR_TC)); /* 确保P发完 */
#endif
                    Dispatch(cmd, u7_tmp, len);
                } else {
#if U7_DEBUG
                    while (!(USART1->SR & USART_SR_TXE));
                    USART1->DR = 'F';
                    while (!(USART1->SR & USART_SR_TC));
#endif
                }
            }
            rx_pos = 0; rx_state = S_HDR0;
            break;
        }
    }
}
