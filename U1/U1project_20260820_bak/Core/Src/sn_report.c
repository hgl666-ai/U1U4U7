#include "sn_report.h"
#include "app.h"                /* MGR_CMD_SN_UPLOAD */
#include "isp_programmer.h"     /* ISP_ReadMemory */
#include "protocol_frame.h"     /* Frame_Build */
#include "usbd_cdc_if.h"        /* CDC_Transmit_FS */
#include <string.h>

/* 上次上传的 UID, 用于去重 (新板才上传) */
static uint8_t last_uid[SN_UID_LEN];

/*===== 内部 =====*/

/**
  * @brief  组帧并发送 SN 上传帧 (U1→PC, 主动, SEQ=0)
  * @param  uid: 96-bit UID (12 字节)
  * @note   A5 5B 00 24 00 00 00 10 [版本][UID12][校验][预留2] CRC16
  */
static void SN_SendFrame(const uint8_t *uid)
{
    uint8_t data[SN_FRAME_LEN];
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t  xor_c = 0;

    data[0] = 0x01;  /* SN 算法版本 */
    memcpy(&data[1], uid, SN_UID_LEN);
    for (uint8_t i = 0; i < SN_UID_LEN; i++) xor_c ^= uid[i];
    data[13] = xor_c;
    data[14] = 0x00;
    data[15] = 0x00;

    uint16_t total = Frame_Build(frame, MGR_CMD_SN_UPLOAD, 0, data, SN_FRAME_LEN);

    uint32_t tick = HAL_GetTick();
    while (CDC_Transmit_FS(frame, total) == USBD_BUSY) {
        if (HAL_GetTick() - tick > 50) break;
    }
}

/*===== 公开 API =====*/

void SN_Report_Init(void)
{
    memset(last_uid, 0xFF, sizeof(last_uid));  /* 首次必不同, 触发上传 */
}

/**
  * @brief  PROGRAM 握手成功后调用: 读 UID → 去重 → 主动上传 SN
  * @note   读 UID 失败时静默跳过, 不中断烧录流程
  */
void SN_Report_OnProgram(void)
{
    uint8_t uid[SN_UID_LEN];

    if (ISP_ReadMemory(SN_UID_BASE, uid, SN_UID_LEN) != ISP_OK) {
        return;  /* 读 UID 失败不中断烧录 */
    }

    /* 同一块板重复烧录不重复上报 */
    if (memcmp(uid, last_uid, SN_UID_LEN) == 0) return;

    memcpy(last_uid, uid, SN_UID_LEN);
    SN_SendFrame(uid);
}
