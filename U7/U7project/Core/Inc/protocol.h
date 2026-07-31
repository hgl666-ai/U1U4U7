#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include <stdint.h>

#define PROTO_MAX_DATA     256

/*===== 命令码 (1 字节, 与 U1 protocol_u7.h 镜像) =====*/
#define PROTO_CMD_PING         0x21
#define PROTO_CMD_GET_VERSION  0x22
#define PROTO_CMD_MOTOR_STEP   0x23
#define PROTO_CMD_MOTOR_STOP   0x24
#define PROTO_CMD_READ_INPUT   0x25
#define PROTO_CMD_SELF_TEST    0x26
#define PROTO_CMD_GET_ADC      0x28
#define PROTO_CMD_RESET        0x27

#define PROTO_DIR_CW    0x00
#define PROTO_DIR_CCW   0x01

#define PROTO_STATUS_OK    0x00
#define PROTO_STATUS_ERROR 0xFF

#define PROTO_FW_MAJOR  1
#define PROTO_FW_MINOR  0

void PROTO_Init(void);
void PROTO_Run(void);

#endif
