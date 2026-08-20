#ifndef __PROTOCOL_FRAME_H
#define __PROTOCOL_FRAME_H

#include "main.h"
#include <stdint.h>

/*===== 帧常量 =====*/
#define FRAME_HDR0        0xA5
#define FRAME_HDR1        0x5B
#define FRAME_HDR_LEN     2
#define FRAME_CMD_LEN     2
#define FRAME_SEQ_LEN     2
#define FRAME_LEN_LEN     2
#define FRAME_CRC_LEN     2
#define FRAME_OVERHEAD    (FRAME_HDR_LEN + FRAME_CMD_LEN + FRAME_SEQ_LEN + FRAME_LEN_LEN + FRAME_CRC_LEN)  /* 10 */
#define FRAME_MAX_DATA    256
#define FRAME_BUF_SIZE    (FRAME_HDR_LEN + FRAME_CMD_LEN + FRAME_SEQ_LEN + FRAME_LEN_LEN + FRAME_MAX_DATA + FRAME_CRC_LEN)

/*===== 1 字节 CMD 帧 (U1↔U7 / U1↔U4) =====*/
#define FRAME_CMD_LEN_1B  1
#define FRAME_OVERHEAD_1B (FRAME_HDR_LEN + FRAME_CMD_LEN_1B + FRAME_SEQ_LEN + FRAME_LEN_LEN + FRAME_CRC_LEN) /* 9 */
#define FRAME_BUF_SIZE_1B (FRAME_HDR_LEN + FRAME_CMD_LEN_1B + FRAME_SEQ_LEN + FRAME_LEN_LEN + FRAME_MAX_DATA + FRAME_CRC_LEN)

/*===== CRC16 (MODBUS, 多项式 0x8005, 初值 0xFFFF) =====*/
uint16_t Frame_CRC16(uint8_t *data, uint16_t len);

/*===== 标准帧 (2B CMD, PC↔U1) =====*/
uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     uint8_t *data, uint16_t len);
uint8_t  Frame_Parse(uint8_t *raw, uint16_t raw_len,
                     uint16_t *cmd, uint16_t *seq,
                     uint8_t *data, uint16_t *len);

/*===== 1 字节 CMD 帧 (U1↔U7 / U1↔U4) =====*/
uint16_t Frame_Build_1B(uint8_t *buf, uint8_t cmd, uint16_t seq,
                        uint8_t *data, uint16_t len);
uint8_t  Frame_Parse_1B(uint8_t *raw, uint16_t raw_len,
                        uint8_t *cmd, uint16_t *seq,
                        uint8_t *data, uint16_t *len);

/*===== 1 字节 CMD 帧, CRC 低字节在前 (U1↔U4 专用, U4 用 MODBUS RTU 序) =====
 * [2026-08-12] U4 固件实测 CRC 低字节在前; U1↔U7 保持高字节在前 */
uint16_t Frame_Build_1B_LE(uint8_t *buf, uint8_t cmd, uint16_t seq,
                           uint8_t *data, uint16_t len);
uint8_t  Frame_Parse_1B_LE(uint8_t *raw, uint16_t raw_len,
                           uint8_t *cmd, uint16_t *seq,
                           uint8_t *data, uint16_t *len);

#endif
