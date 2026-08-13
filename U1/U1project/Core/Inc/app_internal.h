#ifndef __APP_INTERNAL_H
#define __APP_INTERNAL_H

#include "app.h"

/* app.c 提供的共享内部服务 — 供 app 各功能模块使用, 不对外 */

void APP_SetRGB(RGB_Color_t color, uint8_t blinking);
void APP_RGB_Update(void);
void APP_Print(const char *str);
void APP_SendAck(uint16_t cmd, uint8_t *data, uint16_t len);
void APP_SendEvent(uint16_t cmd, uint8_t *data, uint16_t len);
void Flash_ReadHead(uint8_t slot, uint32_t *p_size, uint8_t p_ver[3], uint8_t *p_flag);
uint8_t Flash_WriteCompleteFlag(uint8_t slot, uint8_t flag);

extern APP_State_t app_state;
extern uint16_t pc_seq;

#endif
