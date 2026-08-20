#ifndef __SN_REPORT_H
#define __SN_REPORT_H

#include "main.h"
#include <stdint.h>

/*===== SN 定义 =====*/
#define SN_UID_BASE    0x1FFF7590U   /* STM32G0 96-bit 唯一芯片 ID 基址 */
#define SN_UID_LEN     12U           /* 96-bit = 12 字节 */
#define SN_FRAME_LEN   16U           /* [版本1B][UID 12B][校验1B][预留2B] */

/*===== API =====*/
void SN_Report_Init(void);            /* 初始化去重状态 (app 启动时调用) */
void SN_Report_OnProgram(void);       /* PROGRAM 握手成功后调用: 读UID→去重→上传SN */

#endif
