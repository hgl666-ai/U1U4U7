#ifndef __APP_FRAME_H
#define __APP_FRAME_H

#include "main.h"
#include <stdint.h>

/* PC↔U1 帧接收状态机 (字节流 → 完整帧) */

void     APP_Frame_Init(void);              /* 复位接收状态 */
void     APP_Frame_Feed(uint8_t byte);      /* 接收字节喂入 (APP_USB_Receive 调用) */
uint8_t  APP_Frame_Pending(void);           /* 1=完整帧已收 (含 CRC) */
uint8_t *APP_Frame_Consume(uint16_t *len);  /* 取帧: 清 ready, 返回 buf 指针 + 长度 */

#endif
