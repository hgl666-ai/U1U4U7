#ifndef __PROTOCOL_U7_H
#define __PROTOCOL_U7_H

#include "main.h"
#include <stdint.h>

/* [2026-08-21] U1 侧调试总开关: 1=启用(U7 会话诊断打印, 走 USB CDC), 0=关闭(默认, 不占 Flash)。
 * 改为 1 后需重新编译, 诊断输出在 COM10 可见 */
#ifndef U1_DEBUG
#define U1_DEBUG    0
#endif

#define U7_MAX_DATA     256
#define U7_RETRY_MAX    3
#define U7_RETRY_DELAY  5

/*===== 命令码 (1 字节) =====*/
#define U7_CMD_PING         0x21
#define U7_CMD_GET_VERSION  0x22
#define U7_CMD_MOTOR_STEP   0x23
#define U7_CMD_MOTOR_STOP   0x24
#define U7_CMD_READ_INPUT   0x25
#define U7_CMD_SELF_TEST    0x26
#define U7_CMD_GET_ADC      0x28
#define U7_CMD_HOME         0x29   /* [2026-08-20] 电机回零 (U7 反转找 IN1 零点, 阻塞≤3.2s) */
#define U7_CMD_RESET        0x27

#define U7_MOTOR_CW         0x00
#define U7_MOTOR_CCW        0x01

#define U7_STATUS_OK        0x00
#define U7_STATUS_ERROR     0xFF

#define U7_PROTO_OK             0
#define U7_PROTO_PENDING        1
#define U7_PROTO_ERR_TIMEOUT   -1
#define U7_PROTO_ERR_FRAME     -2
#define U7_PROTO_ERR_CHKSUM    -3
#define U7_PROTO_ERR_BUSY      -4

void U7_Proto_Init(void);
void U7_Proto_Run(void);

int U7_Ping(void);
int U7_GetVersion(uint8_t *major, uint8_t *minor);
int U7_MotorStep(uint8_t motor, uint8_t direction, uint16_t steps);
int U7_MotorStop(uint8_t motor);
int U7_ReadInput(uint8_t index, uint8_t *level);
int U7_SelfTest(uint8_t *result);
int U7_GetADC(uint8_t *buf);   /* buf需16字节, 4通道×4B */
int U7_Reset(void);
int U7_Home(void);             /* [2026-08-20] 电机回零: 等 1B 状态, 禁重试 */

#endif
