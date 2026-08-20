#ifndef __FW_UPLOAD_H
#define __FW_UPLOAD_H

#include "main.h"
#include <stdint.h>

/* U4 固件上传 (CMD 0x0009/0x000A/0x000B): PC → U1 → SPI Flash 双槽 */

void     FwUpload_Init(void);              /* 复位上传状态 */
uint8_t  FwUpload_Handle(uint16_t cmd, uint8_t *data, uint16_t len); /* START/DATA/END, 1=已处理 */
void     FwUpload_TimeoutCheck(void);      /* 10s 无数据超时, APP_Run 周期调用 */

#endif
